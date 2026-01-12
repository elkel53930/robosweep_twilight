#ifndef SPI_MANAGER_H
#define SPI_MANAGER_H

#include <SPI.h>

// 共有SPIオブジェクトの宣言
extern SPIClass shared_spi;

// 他のクラスからSPIバスにアクセスするための関数
SPIClass& get_shared_spi();

// SPI初期化関数
void init_shared_spi();

#endif