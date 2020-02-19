public enum SEQUENCE_INDICATOR {  // enum defined in java. States for the ATE_COMMS
    CALCULATE(0),                         //= 0
    APPLYandLISTEN(1),                    //= 1
    IDLE(2);                              //= 2 (added state for the method3)
    
    private final int value;
    private SEQUENCE_INDICATOR(int value) {
      this.value = value;
    }
    
    // converting the enum to int
    public int getValue() {
      return value;
    }

    // converting int to enum
    public static SEQUENCE_INDICATOR fromInt(int intVal) {
      switch(intVal){
        case 0:
          return CALCULATE;
        case 1: 
          return APPLYandLISTEN;
        case 2:
          return IDLE;
      }
      return null;
    }
}
