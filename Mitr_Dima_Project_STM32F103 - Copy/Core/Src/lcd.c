
//lcd_nokia.h
#include "lcd.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"



// LCD memory index
unsigned int  LcdMemIdx;

// represent LCD matrix
unsigned char  LcdMemory[LCD_CACHE_SIZE];

// simple delay
void Delay(unsigned long a) { while (--a!=0); }


extern SPI_HandleTypeDef hspi1;



/****************************************************************************/
/*  Init LCD Controler                                                      */
/*  Function : LCDInit                                                      */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDInit(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;


  // Enable SPI1 and GPIOA clocks
   
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

    
    /* for SPI1  */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);    
          
  /*Configure GPIO pin : PB2 */
    
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
/*Configure GPIO pin Output Level */
    
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  
  /*Configure GPIO pins : PC7 PC10 */
  
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
  // SPI1 configuration
  
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;

  __HAL_SPI_ENABLE(&hspi1);
  
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    //Error_Handler();
  }

  

  
  // Toggle display reset pin.
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1000);

  // Send sequence of command
  LCDSend( 0x21, SEND_CMD );  // LCD Extended Commands.
  LCDSend( 0xC8, SEND_CMD );  // Set LCD Vop (Contrast).
  LCDSend( 0x06, SEND_CMD );  // Set Temp coefficent.
  LCDSend( 0x13, SEND_CMD );  // LCD bias mode 1:48.
  LCDSend( 0x20, SEND_CMD );  // LCD Standard Commands, Horizontal addressing mode.
  LCDSend( 0x08, SEND_CMD );  // LCD blank
  LCDSend( 0x0C, SEND_CMD );  // LCD in normal mode.

  // Clear and Update

  LCDClear();
  LCDUpdate();

}

/****************************************************************************/
/*  Send to LCD                                                             */
/*  Function : LCDSend                                                      */
/*      Parameters                                                          */
/*          Input   :  data and  SEND_CHR or SEND_CMD                       */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDSend(unsigned char data, unsigned char cd) {

  // Enable display controller (active low) -> LCD_E low
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  // command or data - D/S low or high
  if(cd == SEND_CHR) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  }
  else {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  }

  ///// SEND SPI /////
  // Loop while DR register in not emplty
  
       while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE)==RESET);
  // Send byte through the SPI1 peripheral

       HAL_SPI_Transmit(&hspi1, &data, 1, 100);

  // Wait to receive a byte
       while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE)==RESET);

  // Return the byte read from the SPI bus
  // return SPI_ReceiveData(SPI1);
  ///// SEND SPI END /////


  // Disable display controller -> LCD_E high
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
}

/****************************************************************************/
/*  Update LCD memory                                                       */
/*  Function : LCDUpdate                                                    */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDUpdate ( void )
{

  int i;

  //  Set base address X=0 Y=0
  LCDSend(0x80, SEND_CMD );
  LCDSend(0x40, SEND_CMD );

  //  Serialize the video buffer.
  for (i=0; i<LCD_CACHE_SIZE; i++) {
    LCDSend( LcdMemory[i], SEND_CHR );
  }
}

/****************************************************************************/
/*  Clear LCD                                                               */
/*  Function : LCDClear                                                     */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDClear(void) {

  int i;

  // loop all cashe array
  for (i=0; i<LCD_CACHE_SIZE; i++)
  {
     LcdMemory[i] = 0x0;
  }

}




/****************************************************************************/
/*  Change LCD Pixel mode                                                   */
/*  Function : LcdContrast                                                  */
/*      Parameters                                                          */
/*          Input   :  contrast                                             */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDPixel (unsigned char x, unsigned char y, unsigned char mode )
{
    unsigned int    index   = 0;
    unsigned char   offset  = 0;
    unsigned char   data    = 0;

    // check for out off range
    if ( x > LCD_X_RES ) return;
    if ( y > LCD_Y_RES ) return;

    index = ((y / 8) * 84) + x;
    offset  = y - ((y / 8) * 8);

    data = LcdMemory[index];

    if ( mode == PIXEL_OFF )
    {
        data &= (~(0x01 << offset));
    }
    else if ( mode == PIXEL_ON )
    {
        data |= (0x01 << offset);
    }
    else if ( mode  == PIXEL_XOR )
    {
        data ^= (0x01 << offset);
    }

    LcdMemory[index] = data;

}

/****************************************************************************/
/*  Write char at x position on y row                                       */
/*  Function : LCDChrXY                                                     */
/*      Parameters                                                          */
/*          Input   :  pos, row, char                                       */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDChrXY (unsigned char x, unsigned char y, unsigned char ch )
{
    unsigned int    index   = 0;
    unsigned int    i       = 0;

    // check for out off range
    if ( x > LCD_X_RES ) return;
    if ( y > LCD_Y_RES ) return;

    index = (x*48 + y*48*14)/8 ;

    for ( i = 0; i < 5; i++ )
    {
      LcdMemory[index] = FontLookup[ch - 32][i] << 1;
      index++;
    }

}


/****************************************************************************/
/*  Write char at x position on y row - inverse                             */
/*  Function : LCDChrXY                                                     */
/*      Parameters                                                          */
/*          Input   :  pos, row, char                                       */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDChrXYInverse (unsigned char x, unsigned char y, unsigned char ch )
{
    unsigned int    index   = 0;
    unsigned int    i       = 0;

    // check for out off range
    if ( x > LCD_X_RES ) return;
    if ( y > LCD_Y_RES ) return;

    index = (x*48 + y*48*14)/8 ;

    for ( i = 0; i < 5; i++ )
    {
      LcdMemory[index] = ~(FontLookup[ch - 32][i]);
      index++;

      if(i==4)
        LcdMemory[index] = 0xFF;
    }

}


/****************************************************************************/
/*  Set LCD Contrast                                                        */
/*  Function : LcdContrast                                                  */
/*      Parameters                                                          */
/*          Input   :  contrast                                             */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDContrast(unsigned char contrast) {

    //  LCD Extended Commands.
    LCDSend( 0x21, SEND_CMD );

    // Set LCD Vop (Contrast).
    LCDSend( 0x80 | contrast, SEND_CMD );

    //  LCD Standard Commands, horizontal addressing mode.
    LCDSend( 0x20, SEND_CMD );
}


/****************************************************************************/
/*  Send string to LCD                                                      */
/*  Function : LCDStr                                                       */
/*      Parameters                                                          */
/*          Input   :  row, text, inversion                                 */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDStr(unsigned char row, unsigned char *dataPtr, unsigned char inv ) {

  // variable for X coordinate
  unsigned char x = 0;

  // loop to the and of string
  while ( *dataPtr ) {

    if(inv) {
      LCDChrXYInverse(x, row, (*dataPtr));
    }
    else {
      LCDChrXY( x, row, (*dataPtr));
    }
    x++;
    dataPtr++;
  }
}



