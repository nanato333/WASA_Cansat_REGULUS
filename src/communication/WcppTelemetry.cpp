#include "communication/WcppTelemetry.h"
#include "packet.h"
#include <string.h>
#include <math.h>

namespace WcppTelemetry {
size_t encode(const CanSatData_t &d, uint8_t *buffer, size_t capacity) {
    if (!buffer || capacity < 32 || capacity > 255) return 0;
    memset(buffer, 0, capacity);
    wcpp::Packet packet = wcpp::Packet::empty(buffer, (uint8_t)capacity);
    packet.telemetry(PACKET_ID, COMPONENT_ID);
    bool ok = true;
    ok &= packet.append("TI").setFloat64(d.sys.timestamp / 1000.0);
    ok &= packet.append("AL").setFloat32(d.baro.altitude);
    ok &= packet.append("PR").setFloat32(d.baro.pressure);
    ok &= packet.append("TE").setFloat32(d.baro.temperature);
    ok &= packet.append("ST").setInt(d.sys.phase);
    ok &= packet.append("LA").setFloat64(d.gnss.latitude);
    ok &= packet.append("LO").setFloat64(d.gnss.longitude);
    ok &= packet.append("SA").setInt(d.gnss.satellites);
    ok &= packet.append("AX").setFloat32(d.imu.ax);
    ok &= packet.append("AY").setFloat32(d.imu.ay);
    ok &= packet.append("AZ").setFloat32(d.imu.az);
    ok &= packet.append("GX").setFloat32(d.imu.gx);
    ok &= packet.append("GY").setFloat32(d.imu.gy);
    ok &= packet.append("GZ").setFloat32(d.imu.gz);
    ok &= packet.append("MX").setFloat32(d.mag.mx);
    ok &= packet.append("MY").setFloat32(d.mag.my);
    ok &= packet.append("MZ").setFloat32(d.mag.mz);
    ok &= packet.append("HD").setFloat32(d.mag.heading);
    const float roll = atan2f(d.imu.ay, d.imu.az) * 180.0f / PI;
    const float pitch = atan2f(-d.imu.ax, sqrtf(d.imu.ay*d.imu.ay + d.imu.az*d.imu.az)) * 180.0f / PI;
    ok &= packet.append("OX").setFloat32(pitch);
    ok &= packet.append("OY").setFloat32(roll);
    ok &= packet.append("OZ").setFloat32(d.mag.heading);
    ok &= packet.append("IV").setBool(d.imu.is_valid);
    ok &= packet.append("MV").setBool(d.mag.is_valid);
    ok &= packet.append("BV").setBool(d.baro.is_valid);
    ok &= packet.append("NV").setBool(d.gnss.is_valid);
    ok &= packet.append("FX").setBool(d.gnss.fix);
    if (!ok) return 0;
    const uint8_t payloadSize = packet.size();
    if ((size_t)payloadSize + 1 > capacity || payloadSize >= 250) return 0;
    buffer[0] = payloadSize + 1;
    buffer[payloadSize] = wcpp::Packet::checksum(buffer, payloadSize);
    return payloadSize + 1;
}

bool decodeAction(const uint8_t *buffer, size_t length, uint8_t &action) {
    if (!buffer || length < 5 || length > 250 || buffer[0] != length) return false;
    if (wcpp::Packet::checksum(buffer, length - 1) != buffer[length - 1]) return false;
    wcpp::Packet packet = wcpp::Packet::decode(buffer);
    if (!packet.isCommand()) return false;
    auto entry = packet.find("AC");
    if (entry == packet.end() || !(*entry).isInt()) return false;
    action = (uint8_t)(*entry).getUInt();
    return true;
}
}