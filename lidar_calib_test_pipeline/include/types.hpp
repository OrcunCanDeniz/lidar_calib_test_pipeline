#include <math.h>
#include <string>
#include <algorithm>


struct rotation {
    private:
    float sin_roll, cos_roll;
    float sin_pitch, cos_pitch;
    float sin_yaw, cos_yaw;
    float add_count;

    public:
    float roll;
    float pitch;
    float yaw;

    rotation(float roll=0, float pitch=0, float yaw=0): roll(roll), pitch(pitch), yaw(yaw){}

    rotation& operator=(const rotation& a) //common
    {
        roll = a.roll;
        pitch = a.pitch;
        yaw = a.yaw;
        return *this;
    }

    void operator+=(const rotation& a) // err
    {
        sin_roll += sin(a.roll);
        cos_roll += cos(a.roll); 
        sin_pitch += sin(a.pitch);
        cos_pitch += cos(a.pitch);
        sin_yaw += sin(a.pitch);
        cos_yaw += cos(a.pitch);

        add_count++;
    }

    rotation stDev()
    { //numpy implementation 0 and pi
        rotation ret;

        float mean_sin_roll = sin_roll / add_count;
        float mean_sin_pitch = sin_pitch / add_count;
        float mean_sin_yaw = sin_yaw / add_count;

        float mean_cos_roll = cos_roll / add_count;
        float mean_cos_pitch = cos_pitch / add_count;
        float mean_cos_yaw = cos_yaw / add_count;
        
        float roll_R = std::min(1.0f, (float)hypot(mean_sin_roll, mean_cos_roll));
        float pitch_R = std::min(1.0f, (float)hypot(mean_sin_pitch, mean_cos_pitch));
        float yaw_R = std::min(1.0f, (float)hypot(mean_sin_yaw, mean_cos_yaw));
        
        ret.roll = sqrt(-2 * log(roll_R)) ;
        ret.pitch = sqrt(-2 * log(pitch_R)) ;
        ret.yaw = sqrt(-2 * log(yaw_R)) ;
        
        return ret;
    }

    rotation mean()
    {
        rotation ret;
        
        ret.roll = atan2(sin_roll , cos_roll);
        ret.pitch = atan2(sin_pitch , cos_pitch);
        ret.yaw = atan2(sin_yaw , cos_yaw);

        return ret;
    }
};

struct translation {
    private:
        float add_count = 0;
        void rollingSum(translation other)
        {
            x += other.x;
            y += other.y;
            z += other.z;
        }
        void addDiff(translation other)
        { // add (t-t')^2 to x,y,z respectively

        }

    public:

        float x;
        float y;
        float z;

        translation(float x=0, float y=0, float z=0): x(x), y(y), z(z){}

        translation& operator=(const translation& a) //common
        {
            x = a.x;
            y = a.y;
            z = a.z;
            return *this;
        }

        void operator+=(translation a)
       {
           x += a.x;
           y += a.y;
           z += a.z;
// https://www.strchr.com/standard_deviation_in_one_pass
           add_count++;
       }

        translation& mean() 
        {
            translation ret;
            ret.x /= add_count; 
            ret.y /= add_count; 
            ret.z /= add_count; 
            return ret;
        }
};


struct genericT 
{
    public:
        translation trans;
        rotation rot;
    
    private:
        float sin_roll, cos_roll;
        float sin_pitch, cos_pitch;
        float sin_yaw, cos_yaw;
        float add_count;

    genericT( float x=0, float y=0, float z=0, float roll=0, float pitch=0, float yaw=0, float access_count=0) 
        : sin_roll(0), cos_roll(0), sin_pitch(0), cos_pitch(0), sin_yaw(0), cos_yaw(0), add_count(0)
    {
        trans.x = x; 
        trans.y = y; 
        trans.z = z; 
        rot.roll = roll; 
        rot.pitch = pitch; 
        rot.yaw = yaw;   
    }

    genericT& operator=(const genericT& a) //common
    {
        trans = a.trans;
        rot = a.rot;
        return *this;
    }

    genericT operator/(const float& scalar) const 
    {
        return genericT(trans.x/scalar, trans.y/scalar, trans.z/scalar, rot.roll/scalar, rot.pitch/scalar, rot.yaw/scalar);
    }

    void operator+=(const genericT& a) // err
    {

        rot += a.rot;
        trans += a.trans;

        add_count++;
    }

    genericT getMean()
    {
        genericT ret;
        ret.rot = rot.mean();
        ret.trans = trans.mean();
    }

    genericT getStDev()
    {
        genericT ret;
        ret.rot = rot.stDev();
        ret.trans = trans.stDev();
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



// genericT operator+(const genericT& a) const 
// {
//     return genericT(a.x+x, a.y+y, a.z+z, a.roll+roll, a.pitch+pitch, a.yaw+yaw);
// }


// struct errT:genericT 
// {
//     void operator+=(const genericT& a) // err
//     {
//         x+=a.x;
//         y+=a.y;
//         z+=a.z;
//         roll += a.roll;
//         pitch += a.pitch;
//         yaw += a.yaw;
//     }
// };


// struct tfT:genericT 
// {
//     void operator+=(const tfT& a) // tf 
//     {
//         x+=a.x;
//         y+=a.y;
//         z+=a.z;
//         roll += sin(roll) + sin(a.roll), cos(roll) + cos(a.roll);
//         pitch += sin(pitch) + sin(a.pitch), cos(pitch) + cos(a.pitch);
//         yaw += sin(yaw) + sin(a.yaw), cos(yaw) + cos(a.yaw);
//     }
// };

// genericT getCircStd()
// { //numpy implementation 0 and pi
//     genericT ret;
//     ret.x = x / add_count;
//     ret.y = y / add_count;
//     ret.z = z / add_count;

//     float mean_sin_roll = sin_roll / add_count;
//     float mean_sin_pitch = sin_pitch / add_count;
//     float mean_sin_yaw = sin_yaw / add_count;

//     float mean_cos_roll = cos_roll / add_count;
//     float mean_cos_pitch = cos_pitch / add_count;
//     float mean_cos_yaw = cos_yaw / add_count;
    
//     float roll_R = std::min(1.0f, hypot(mean_sin_roll, mean_cos_roll));
//     float pitch_R = std::min(1.0f, hypot(mean_sin_pitch, mean_cos_pitch));
//     float yaw_R = std::min(1.0f, hypot(mean_sin_yaw, mean_cos_yaw));
    
//     ret.roll = sqrt(-2 * log(roll_R)) ;
//     ret.pitch = sqrt(-2 * log(pitch_R)) ;
//     ret.yaw = sqrt(-2 * log(yaw_R)) ;
    
//     return ret;



// sqrt(-2 * log(sqrt(sum(Sin2)^2 + sum(Cos2)^2) / length(Rad2)))