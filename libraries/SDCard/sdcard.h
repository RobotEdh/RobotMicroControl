#ifndef SDCARD_h
#define SDCARD_h

#include <SPI.h>
#include <SD.h>   

#define SUCCESS 0
#define SDCARD_ERROR -7
#define FILE_OPEN_ERROR  -1000
#define FILE_CLOSE_ERROR -1001

#define SS_CS_Pin  10   

int initSDCard(void);
/* Description: Initialization of the SD-Card                                 */                                            
/* input:       none                                                          */                      
/* output:      return                                                        */                            
/*                  = SDCARD_ERROR if error during initialization SD-Card     */
/*                  = SUCCESS otherwise                                       */ 
/* lib:         card.init                                                     */
/*              volume.init                                                   */
/*              root.openRoot                                                 */
        
int infoSDCard(void);
/* Description: Get infos from the SD-Card                                    */                                            
/* input:       none                                                          */                      
/* output:      return                                                        */                            
/*                  = SDCARD_ERROR if error during reading CID of the SD-Card */
/*                  = SUCCESS otherwise                                       */ 
/* lib:         card.readCID                                                  */
/*              root.ls                                                       */      

#endif 