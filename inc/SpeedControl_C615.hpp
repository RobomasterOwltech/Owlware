#include "BaseMotor.hpp"

class SpeedControl_C615 : BaseMotor {
   private:
      PWM_Control controller;
   public:
      Motor_2305();
      ~Motor_2305();
      void setReference();
      void getFeedback();
};

