public enum receive_Stat {  // enum defined in java. States for the ATE_COMMS
    CURRENT(0),                       //= 0
    POSITION(1),                      //= 1
    TORQUE(2),                        //= 2
    SPEED(3),                         //= 3
    TIMER(4),
    INPUT(5);
    
    private final int value;
    private receive_Stat(int value) {
      this.value = value;
    }
    
    // converting the enum to int
    public int getValue() {
      return value;
    }

       
}
