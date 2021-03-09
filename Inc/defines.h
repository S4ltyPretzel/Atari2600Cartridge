/* Enable SDIO/SDMMC with 1-bit communication */
#define FATFS_SDIO_4BIT   0
/* Enable SPI communication, disable SDIO/SDMMC */
#define FATFS_USE_SDIO       0

/* Overwrite default SPI settings if needed */
#define FATFS_SPI            SPI1
#define FATFS_SPI_PINSPACK   TM_SPI_PinsPack_2

/* Set your CS pin for SPI */
#define FATFS_CS_PORT        GPIOA
#define FATFS_CS_PIN         GPIO_PIN_15
