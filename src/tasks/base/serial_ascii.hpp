#pragma once

// 视觉 <-> 下位机 ASCII 串口协议
//
//   视觉 -> 下位机:  AAP<pitch>Y<yaw>F<fire>N<frame>C<crc>%<LF>
//                    AAP1.23Y-4.56F0N120Cc328%
//   下位机 -> 视觉:  ARQ<w>X<x>Y<y>Z<z>C<crc>%<LF>
//                    ARQ0.7071X0.0000Y0.7071Z0.0000C2dd9%
//
// pitch/yaw 为度、增量（相对当前云台角）；fire 0/1（1 = 连发）；
// crc 为 CRC16-MCRF4XX（初值 0xFFFF、反射多项式 0x8408），
// 覆盖 'C' 之前的整段 ASCII，输出 4 位小写十六进制。
// 接收侧末尾的 '%' 可省略（允许直接以 \r 或 \n 结尾）。

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>

namespace awakening::serial_ascii {

// 反射式 CRC16（初值 0xFFFF），poly 为反射后的多项式：
//   MCRF4XX = 0x8408（固件实际使用）
//   Modbus  = 0xA001（早期文档写的，接收端也兼容）
inline uint16_t crc16_reflected(std::string_view data, uint16_t poly) {
    uint16_t crc = 0xFFFF;
    for (const unsigned char byte: data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc = static_cast<uint16_t>((crc >> 1) ^ poly);
            } else {
                crc = static_cast<uint16_t>(crc >> 1);
            }
        }
    }
    return crc;
}

inline uint16_t crc16_mcrf4xx(std::string_view data) {
    return crc16_reflected(data, 0x8408);
}

inline uint16_t crc16_modbus(std::string_view data) {
    return crc16_reflected(data, 0xA001);
}

inline bool hex4_to_u16(std::string_view text, uint16_t& out) {
    if (text.size() != 4) {
        return false;
    }
    uint16_t value = 0;
    for (const char c: text) {
        int digit = 0;
        if (c >= '0' && c <= '9') {
            digit = c - '0';
        } else if (c >= 'a' && c <= 'f') {
            digit = c - 'a' + 10;
        } else if (c >= 'A' && c <= 'F') {
            digit = c - 'A' + 10;
        } else {
            return false;
        }
        value = static_cast<uint16_t>((value << 4) | digit);
    }
    out = value;
    return true;
}

// 视觉 -> 下位机的一帧
inline std::string encode_aap(double pitch_deg, double yaw_deg, bool fire, uint32_t frame) {
    char head[96];
    const int n = std::snprintf(
        head,
        sizeof(head),
        "AAP%.2fY%.2fF%dN%u",
        pitch_deg,
        yaw_deg,
        fire ? 1 : 0,
        frame
    );
    std::string out(head, n > 0 ? static_cast<size_t>(n) : 0);

    char tail[8];
    std::snprintf(tail, sizeof(tail), "C%04x%%\n", crc16_mcrf4xx(out));
    out += tail;
    return out;
}

struct ArqFrame {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 流式组帧：串口上可能混有其它二进制包，必须按 ARQ 帧头 + 帧尾 + CRC 切帧，
// 并且允许一帧跨多个读块。
class ArqFramer {
public:
    template<typename Fn>
    void feed(std::string_view chunk, Fn&& on_frame) {
        buf_.append(chunk.data(), chunk.size());
        if (buf_.size() > MAX_BUFFER) {
            buf_.erase(0, buf_.size() - MAX_BUFFER);
        }

        for (;;) {
            const auto start = buf_.find("ARQ");
            if (start == std::string::npos) {
                // 保留末尾 2 个字符，避免帧头被读块切断
                if (buf_.size() > 2) {
                    buf_.erase(0, buf_.size() - 2);
                }
                return;
            }
            if (start > 0) {
                buf_.erase(0, start);
            }

            const auto end = buf_.find_first_of("%\r\n", 3);
            if (end == std::string::npos) {
                if (buf_.size() > MAX_FRAME) {
                    // 帧太长还没收全，丢掉帧头重新同步
                    buf_.erase(0, 3);
                    continue;
                }
                return; // 等下一个读块
            }

            ArqFrame parsed;
            if (parse_arq(std::string_view(buf_.data(), end), parsed)) {
                ++good_frames_;
                on_frame(parsed);
            } else {
                ++bad_frames_;
            }
            buf_.erase(0, end + 1);
        }
    }

    [[nodiscard]] uint64_t good_frames() const noexcept {
        return good_frames_;
    }
    [[nodiscard]] uint64_t bad_frames() const noexcept {
        return bad_frames_;
    }

private:
    static constexpr size_t MAX_FRAME = 128;
    static constexpr size_t MAX_BUFFER = 4096;

    static bool parse_arq(std::string_view frame, ArqFrame& out) {
        // frame 形如 ARQ<w>X<x>Y<y>Z<z>C<crc>，不含帧尾
        if (frame.size() < 8 || frame.substr(0, 3) != "ARQ") {
            return false;
        }
        if (frame[frame.size() - 5] != 'C') {
            return false;
        }

        uint16_t want = 0;
        if (!hex4_to_u16(frame.substr(frame.size() - 4), want)) {
            return false;
        }
        // 接收方向两种 CRC 都认（固件/文档不一致时避免整段丢帧）
        const auto payload = frame.substr(0, frame.size() - 5);
        if (crc16_mcrf4xx(payload) != want && crc16_modbus(payload) != want) {
            return false;
        }

        const char* cursor = frame.data() + 3;
        const char* const end = frame.data() + frame.size() - 5;
        char* next = nullptr;

        const double w = std::strtod(cursor, &next);
        if (next == cursor) {
            return false;
        }
        cursor = next;

        constexpr char tags[3] = { 'X', 'Y', 'Z' };
        double values[3] = { 0.0, 0.0, 0.0 };
        for (int i = 0; i < 3; ++i) {
            if (cursor >= end || *cursor != tags[i]) {
                return false;
            }
            ++cursor;
            values[i] = std::strtod(cursor, &next);
            if (next == cursor) {
                return false;
            }
            cursor = next;
        }

        out = ArqFrame { w, values[0], values[1], values[2] };
        return true;
    }

    std::string buf_;
    uint64_t good_frames_ = 0;
    uint64_t bad_frames_ = 0;
};

} // namespace awakening::serial_ascii
