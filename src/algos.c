#include "main.h"
#include "algos.h"

void PhaseChange(uint8_t Phases){
	uint8_t PhaseA = (Phases & 0b000011);
	uint8_t PhaseB = (Phases & 0b001100) >> 2;
	uint8_t PhaseC = (Phases & 0b110000) >> 4;
	int Registers = 0;
	Registers |= (LAA1_Pin << (16-(PhaseA&0b01)*16)) | (LAA2_Pin << (16-(PhaseA&0b10)*16));
	Registers |= (LBB1_Pin << (16-(PhaseB&0b01)*16)) | (LBB2_Pin << (16-(PhaseB&0b10)*16));
	Registers |= (LCC1_Pin << (16-(PhaseC&0b01)*16)) | (LCC2_Pin << (16-(PhaseC&0b10)*16));

	GPIOB->BSRR = Registers; 
}

void sortThree(float32_t arr[3]) {
    float32_t a = arr[0], b = arr[1], c = arr[2];
    
    if (a > b) {
        if (a > c) {
            if (b > c) {
                arr[0] = c; arr[1] = b; arr[2] = a;
            } else {
                arr[0] = b; arr[1] = c; arr[2] = a;
            }
        } else {
            arr[0] = b; arr[1] = a; arr[2] = c;
        }
    } else {
        if (b > c) {
            if (a > c) {
                arr[0] = c; arr[1] = a; arr[2] = b;
            } else {
                arr[0] = a; arr[1] = c; arr[2] = b;
            }
        }
    }
}
