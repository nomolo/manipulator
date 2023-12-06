#include "display_data.h"

mnplt::DataDisplay::DataDisplay() {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);

    // 向服务器（特定的IP和端口）发起请求
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));  // 每个字节都用0填充
    serv_addr.sin_family = AF_INET;            // 使用IPv4地址
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");  // 具体的IP地址
    serv_addr.sin_port = htons(HTONS_);                  // 端口
    if (connect(sock_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) != -1) {
    }
}

void mnplt::DataDisplay::setData(std::string data_name, double data) {
    Json::Value root;
    root["timestamp"] = time;
    root["data"][data_name] = data;
    Json::FastWriter fastWriter;
    auto str = fastWriter.write(root);
    write(sock_, str.data(), str.length());
}
