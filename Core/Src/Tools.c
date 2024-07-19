#include "Tools.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

void LogNumberToPC(int Num)
{
	char str[100];
	sprintf(str, "Result is %2d\n", Num);
	HAL_UART_Transmit(&huart1, str, strlen(str), 100);
}

void LogStringToPC(char *Data)
{
	HAL_UART_Transmit(&huart1, Data, strlen(Data), 1000);
	HAL_UART_Transmit(&huart1, "\n", 1, 1000);
}

void LogFLStringToPC(char *Data, int Length)
{
	HAL_UART_Transmit(&huart1, Data, Length, 1000);
	HAL_UART_Transmit(&huart1, "\n", 1, 1000);
}

void LogBinaryToPC(int Data)
{
	char *binValue;
	binValue = int_to_binary_string(Data);
	HAL_UART_Transmit(&huart1, binValue, 8, 1000);
	HAL_UART_Transmit(&huart1, "\n", 1, 1000);
}

void LogError(int Code, char *Message)
{
	char Packet[100];
	memset(Packet,'\0', sizeof(Packet));
	sprintf(Packet, "Error = {\"Code\": %d, \"Text\": \"%s\"}", Code, Message);
	LogStringToPC(Packet);
}

void LogInformation(int Code, char *Message)
{
	char Packet[100];
	memset(Packet,'\0', sizeof(Packet));
	sprintf(Packet, "Information = {\"Code\": %d, \"Text\": \"%s\"}", Code, Message);
	LogStringToPC(Packet);
}

char *int_to_binary_string(int num) {
	int INT_BITS = 8;
    char *bin_str = malloc(INT_BITS + 1);  // allocate space for binary string
    int i;
    for (i = 0; i < INT_BITS; i++) {
        bin_str[i] = (num & (1 << (INT_BITS - i - 1))) ? '1' : '0';  // get i-th bit
    }
    bin_str[i] = '\0';  // add null terminator
    return bin_str;
}