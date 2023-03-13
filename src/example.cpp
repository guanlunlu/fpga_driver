# include <stdio.h>
# include <dlfcn.h>
# include <iostream>
# include <NiFpga_FPGA_CANBus_IMU_4module_IRQ.h>
# include <malloc.h>

int main(){
    printf("initializaing...\n");
    NiFpga_Status status = NiFpga_Initialize();

    if (NiFpga_IsNotError(status)){
        NiFpga_Session session;
        /* opens a session, downloads the bitstream, and runs the FPGA */
        printf("Opening a session...\n");

        NiFpga_MergeStatus(&status, NiFpga_Open(NiFpga_FPGA_CANBus_IMU_4module_IRQ_Bitfile, NiFpga_FPGA_CANBus_IMU_4module_IRQ_Signature,"RIO0", NiFpga_OpenAttribute_NoRun, &session));
        
        if (NiFpga_IsNotError(status)){
            /* allocate size for the samples to read */
            uint8_t data[8];
            uint8_t tx_data[8] = {0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFA};

            /* run the FPGA application */
            printf("Running the FPGA...\n");
            NiFpga_MergeStatus(&status, NiFpga_Run(session, 0));
            NiFpga_MergeStatus(&status, NiFpga_ReadArrayU8(session, NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8_Mod1CAN0ID1RXData, data, NiFpga_FPGA_CANBus_IMU_4module_IRQ_IndicatorArrayU8Size_Mod1CAN0ID1RXData));           
            NiFpga_MergeStatus(&status, NiFpga_WriteArrayU8(session, NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8_Mod1CAN0ID1TXData, tx_data, NiFpga_FPGA_CANBus_IMU_4module_IRQ_ControlArrayU8Size_Mod1CAN0ID1TXData));
            int cnt = 0;
            while(1){
                for (int i = 0; i < 8; i++){
                    printf("data[%d]: %d\n",i, data[i]);
                }
                printf("---\n");
                cnt ++;
            }
            /* must close if we successfully opened */
            NiFpga_MergeStatus(&status, NiFpga_Close(session, 0));
        }
        /* must be called after all other calls */
        printf("Finalizing...\n");
        NiFpga_MergeStatus(&status, NiFpga_Finalize());

    }

    if (NiFpga_IsError(status))
    {
        printf("Error %d!\n", status);
        printf("Press <Enter> to quit...\n");
        getchar();
    }

    return status;
}
