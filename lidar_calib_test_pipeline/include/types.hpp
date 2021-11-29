#include <math.h>
#include <string>


struct genericT {
    public:
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float yaw;

    genericT( float x=0, float y=0, float z=0, float roll=0, float pitch=0, float yaw=0, float access_count=0) 
        : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) //common
    {}

    genericT& operator=(const genericT& a) //common
    {
        x=a.x;
        y=a.y;
        z=a.z;
        roll = a.roll;
        pitch = a.pitch;
        yaw = a.yaw;
        return *this;
    }

    genericT operator+(const genericT& a) const 
    {
        return genericT(a.x+x, a.y+y, a.z+z, a.roll+roll, a.pitch+pitch, a.yaw+yaw);
    }

    genericT operator/(const float& scalar) const 
    {
        return genericT(x/scalar, y/scalar, z/scalar, roll/scalar, pitch/scalar, yaw/scalar);
    }
};


struct errT:genericT 
{
    void operator+=(const genericT& a) // err
    {
        x+=a.x;
        y+=a.y;
        z+=a.z;
        roll += a.roll;
        pitch += a.pitch;
        yaw += a.yaw;
    }
};


struct tfT:genericT 
{
    void operator+=(const tfT& a) // tf 
    {
        x+=a.x;
        y+=a.y;
        z+=a.z;
        roll += atan2( sin(roll) + sin(a.roll), cos(roll) + cos(a.roll) );
        pitch += atan2( sin(pitch) + sin(a.pitch), cos(pitch) + cos(a.pitch) );
        yaw += atan2( sin(yaw) + sin(a.yaw), cos(yaw) + cos(a.yaw) );
    }
};


typedef std::pair<genericT, genericT> err_tf_pair;

struct statType 
{
    genericT mean;
    genericT dev;
};

struct stats 
{    
    statType tf;
    statType err;
};

sqrt(-2 * log(sqrt(sum(Sin2)^2 + sum(Cos2)^2) / length(Rad2)))