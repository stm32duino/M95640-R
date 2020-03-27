# M95640-R
Arduino library to support the M95640-R SPI EEPROM

## API

This sensor uses SPI to communicate.

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    dev_spi = new SPIClass(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi->begin();

An instance can be created and enabled with SPI bus following the procedure below:  

    myM95640R = new M95640R(devSPI, D5);
    myM95640R->begin();

If you want to read the EEPROM you can call the following API:

    myM95640R->EepromRead(0x0000, 32, tmpBuffer);

If you want to write data into the EEPROM you can call the following API:

    myM95640R->EepromWrite(0x0000, 32, tmpBuffer);

## Documentation

You can find the source files at  
https://github.com/stm32duino/M95640-R

The M95640-R EEPROM datasheet is available at  
https://www.st.com/content/st_com/en/products/memories/serial-eeprom/standard-serial-eeprom/standard-spi-eeprom/m95640-r.html
