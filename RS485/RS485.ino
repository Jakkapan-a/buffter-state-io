#include <ModbusRTUSlave.h>

// bufferInput class boolean xxx state
class bufferInput{
    public:
        bufferInput(int size){
            for(int i = 0; i < size; i++){
                state[i] = false;
            }
            currentSize = size;
        }

        void set(int index, boolean value){
            state[index] = value;
        }
    private:
        int currentSize = 0;
        boolean state[];
}


void setup() {

}

void loop() {

}
