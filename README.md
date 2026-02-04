    // allocate_mem for th data 
    aht10_data_t *TH_data = malloc(sizeof(aht10_data_t));
    if(!TH_data) {
        printf("malloc failed /n");
    }


    // initialize i2c
    i2c_init();
    vTaskDelay(pdMS_TO_TICKS(75));
    // initialize aht10 with soft reset
    aht10_cmd_init();


    xTaskCreate(read_aht10_task , "aht10_task" , 4096 ,  (void *)TH_data , 1 , NULL);
    
    