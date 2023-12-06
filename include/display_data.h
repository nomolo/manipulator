#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#ifdef __cplusplus
}
#endif

#include "jsoncpp/json/json.h"

#define HTONS_ 9870

namespace mnplt {
class DataDisplay {
   private:
    int sock_;

   public:
    double time;
    DataDisplay();
    ~DataDisplay() { close(sock_); }
    void setData(std::string data_name, double data);
    void show();
};
};  // namespace mnplt