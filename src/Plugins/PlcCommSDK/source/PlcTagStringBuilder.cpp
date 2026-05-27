#include "PlcTagStringBuilder.h"

#include <cctype>
#include <sstream>

namespace {

constexpr uint16_t kDefaultModbusPort = 502;
constexpr int kModbusUnitIdDefault = 1;

std::string trimCopy(std::string s)
{
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
        s.erase(s.begin());
    }
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
        s.pop_back();
    }
    return s;
}

std::string urlEncodeMinimal(const std::string& value)
{
    std::ostringstream oss;
    for (unsigned char ch : value) {
        if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9')
            || ch == '-' || ch == '_' || ch == '.' || ch == ',' || ch == '/') {
            oss << static_cast<char>(ch);
        } else {
            oss << '%' << std::hex << std::uppercase << static_cast<int>(ch);
        }
    }
    return oss.str();
}

bool isAllDigits(const std::string& s)
{
    if (s.empty()) {
        return false;
    }
    for (unsigned char ch : s) {
        if (!std::isdigit(ch)) {
            return false;
        }
    }
    return true;
}

bool hasModbusPrefix(const std::string& lower)
{
    return lower.size() >= 3
        && (lower.rfind("co", 0) == 0 || lower.rfind("di", 0) == 0
            || lower.rfind("hr", 0) == 0 || lower.rfind("ir", 0) == 0);
}

/// libplctag Modbus name：co/di/hr/ir + 从 0 起的寄存器号
bool normalizeModbusTagName(const std::string& input, std::string* outName, std::string* outError)
{
    const std::string trimmed = trimCopy(input);
    if (trimmed.empty()) {
        *outError = "标签名为空";
        return false;
    }

    std::string lower = trimmed;
    for (char& ch : lower) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }

    if (hasModbusPrefix(lower)) {
        const std::string suffix = lower.substr(2);
        if (!isAllDigits(suffix)) {
            *outError = "Modbus 名称格式无效，示例 hr0、40001";
            return false;
        }
        const int reg = std::stoi(suffix);
        if (reg < 0 || reg > 65535) {
            *outError = "寄存器号须在 0–65535";
            return false;
        }
        *outName = lower;
        return true;
    }

    if (!isAllDigits(trimmed)) {
        *outError = "Modbus 须为 40001 式地址或 hr0/co0 等前缀名";
        return false;
    }

    const int addr = std::stoi(trimmed);
    if (addr < 0 || addr > 65535) {
        *outError = "地址超出范围";
        return false;
    }

    std::ostringstream name;
    // 5 位区号：40001–49999 保持寄存器等
    if (addr >= 40001 && addr <= 49999) {
        name << "hr" << (addr - 40001);
    } else if (addr >= 30001 && addr <= 39999) {
        name << "ir" << (addr - 30001);
    } else if (addr >= 10001 && addr <= 19999) {
        name << "di" << (addr - 10001);
    } else if (addr >= 4001 && addr <= 4999) {
        name << "hr" << (addr - 4001);
    } else if (addr >= 3001 && addr <= 3999) {
        name << "ir" << (addr - 3001);
    } else if (addr >= 1001 && addr <= 1999) {
        name << "di" << (addr - 1001);
    } else if (addr >= 1 && addr <= 9999) {
        name << "co" << (addr - 1);
    } else if (addr == 0) {
        name << "co0";
    } else {
        *outError = "无法识别的 Modbus 地址区";
        return false;
    }

    *outName = name.str();
    return true;
}

/// Modbus path = 从站单元 ID（0–255），libplctag 必填
std::string modbusUnitIdPath(const PlcConnectionConfig& connection)
{
    std::string raw = trimCopy(connection.path);
    if (raw.empty()) {
        return std::to_string(kModbusUnitIdDefault);
    }
    const auto comma = raw.find(',');
    if (comma != std::string::npos) {
        raw = trimCopy(raw.substr(0, comma));
    }
    if (!isAllDigits(raw)) {
        return std::to_string(kModbusUnitIdDefault);
    }
    const int unit = std::stoi(raw);
    if (unit < 0 || unit > 255) {
        return std::to_string(kModbusUnitIdDefault);
    }
    return raw;
}

} // namespace

bool buildPlcTagAttributeString(
    const PlcConnectionConfig& connection,
    const PlcTagSpec& tag,
    std::string* outAttributeString,
    std::string* outError)
{
    if (outAttributeString) {
        outAttributeString->clear();
    }
    if (outError) {
        outError->clear();
    }

    std::ostringstream oss;
    oss << std::dec;
    if (connection.protocol == PlcProtocol::AbEip) {
        oss << "protocol=ab_eip"
            << "&gateway=" << urlEncodeMinimal(connection.gateway)
            << "&path=" << urlEncodeMinimal(connection.path)
            << "&cpu=" << urlEncodeMinimal(connection.cpu.empty() ? "lgx" : connection.cpu)
            << "&name=" << urlEncodeMinimal(tag.name);
        if (tag.elemCount > 0) {
            oss << "&elem_count=" << tag.elemCount;
        }
        if (tag.elemSize > 0) {
            oss << "&elem_size=" << tag.elemSize;
        }
    } else {
        std::string modbusName;
        std::string nameError;
        if (!normalizeModbusTagName(tag.name, &modbusName, &nameError)) {
            if (outError) {
                *outError = nameError;
            }
            return false;
        }

        const uint16_t port = connection.port == 0 ? kDefaultModbusPort : connection.port;
        oss << "protocol=modbus_tcp"
            << "&gateway=" << urlEncodeMinimal(connection.gateway)
            << "&port=" << port
            << "&path=" << modbusUnitIdPath(connection)
            << "&name=" << urlEncodeMinimal(modbusName);
        if (tag.elemCount > 0) {
            oss << "&elem_count=" << tag.elemCount;
        }
    }
    if (outAttributeString) {
        *outAttributeString = oss.str();
    }
    return true;
}
