#ifndef _ENCODER_H_
#define _ENCODER_H_

#define PREV_MASK 0x1 // Mask for the previous state in determining direction of rotation.
#define CURR_MASK 0x2 // Mask for the current state in determining direction of rotation.
#define INVALID 0x3 // XORing two states where both bits have changed.

class Encoder {

  public:
    Encoder(int a_pin, int b_pin);
    int getPulses();
    void reset();

  private:
    static void isr();
    static Encoder * instance_;
    
    void encode();

    int a_pin_;
    int b_pin_;

    int prevState_;
    int currState_;
    volatile int pulses_;
};

Encoder::Encoder(int a_pin, int b_pin): a_pin_(a_pin), b_pin_(b_pin)
{
  instance_ = this;
  
  pinMode(a_pin_, INPUT_PULLUP);
  pinMode(b_pin_, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(a_pin_), isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(b_pin_), isr, CHANGE);
  
  int chanA = digitalRead(a_pin_);
  int chanB = digitalRead(b_pin_);

  currState_ = (chanA << 1) | (chanB);
  prevState_ = currState_;
  pulses_ = 0;
}

void Encoder::reset()
{
  pulses_ = 0;
}

int Encoder::getPulses()
{
  return pulses_;
}

void Encoder::isr()
{
  instance_->encode();
}

Encoder * Encoder::instance_;

void Encoder::encode()
{
  int change = 0;
  int chanA = digitalRead(a_pin_);
  int chanB = digitalRead(b_pin_);
  currState_ = (chanA << 1) | (chanB);

  if (((currState_ ^ prevState_) != INVALID) && (currState_ != prevState_))
  {
    change = (prevState_ & PREV_MASK) ^ ((currState_ & CURR_MASK) >> 1);
    if (change == 0)
    {
      change = -1;
    }
    pulses_ -= change;
  }

  prevState_ = currState_;
}

#endif
