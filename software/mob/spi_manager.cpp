#include "spi_manager.h"

// 共有SPIオブジェクトの定義
SPIClass shared_spi(HSPI);

SPIClass& get_shared_spi() {
    return shared_spi;
}

void init_shared_spi() {
    shared_spi.begin(11, 9, 10, -1);  // SCK=11, MISO=9, MOSI=10, CS=-1(software control)
}