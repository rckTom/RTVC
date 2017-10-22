#include <Dataflash.h>

int buffer_counter=0;
int page_counter=0;

int lastpage=0; //last page written to
int pages=8; //total pages that will be used
Dataflash dflash; 

void setup()
{
  Serial.begin(115200);
  Serial.print('h');
  Serial.print('i');
  Serial.print('\n'); //debug
  dflash.init(); //initialize the memory (pins are defined in dataflash.cpp

  int j = 0;
  int i = 0;
  
 
  while (lastpage<=pages)
  {
    for (buffer_counter=0; buffer_counter<528; buffer_counter++)
    {
      dflash.Buffer_Write_Byte(1,buffer_counter,lastpage); //write to buffer 1, 1 byte at a time
    }
  
    Serial.println("buffer written");
    dflash.Buffer_To_Page(1, lastpage);//write buffer 1 to the memory on page: lastpage
    Serial.println("buffer copied to page");
    lastpage++;

    /* for (buffer_counter=0;buffer_counter <528;buffer_counter++)
    {
      dflash.Buffer_Write_Byte(2,buffer_counter,55); //write to buffer 2, 1 byte at a time
    }
 
    dflash.Buffer_To_Page(2, lastpage);//write buffer 2 to the memory on page: lastpage
    lastpage++; */
  }

  for (int i=0;i<=pages;i++)
  {
  dflash.Page_To_Buffer(i, 1);//copy page i to the buffer
    Serial.print("page ");
    Serial.println(i);
    for(int j=0;j<528;j++) //j depends on the amount of data on the page
    {
      Serial.print(dflash.Buffer_Read_Byte(1, j)); //print the buffer data to the serial port
      Serial.print(" ");
    }
    Serial.print('\n');//debug 
  } 
}

void loop()
{

}