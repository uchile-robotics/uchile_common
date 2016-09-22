
#include <ros.h>
#include <bender_turning_base/platoMov.h>
#include <Servo.h>

ros::NodeHandle  nh;

Servo myservo;

void messageCb( const bender_turning_base::platoMov& msg)
  {
    myservo.write(89);
    delay((msg.revolution_time*msg.angle)/360);
    myservo.write(92); 
  }

ros::Subscriber<bender_turning_base::platoMov> sub("toggle_servo", &messageCb );

void setup()
{ 
  myservo.attach(9);
  myservo.write(92); //servo.write(92) -- servo quieto
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

