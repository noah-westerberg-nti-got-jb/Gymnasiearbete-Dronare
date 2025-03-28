#ifndef SMOOTHVALUE_H_
#define SMOOTHVALUE_H_

class SmoothValue {
    float value;
    float smoothingFactor;
public:
    SmoothValue(float smoothingFactor);
    SmoothValue(float smoothingFactor, float initialValue);
    
    operator float() const;
    
    SmoothValue& operator=(float newValue);
    
    SmoothValue& operator+=(float increment);

    SmoothValue& operator-=(float decrement);

    SmoothValue& operator*=(float factor);

    SmoothValue& operator/=(float divisor);

    void set(float newValue);
};
#endif