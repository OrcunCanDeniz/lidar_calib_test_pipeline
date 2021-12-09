#pragma once
#include <math.h>
#include <algorithm>

/* 
    Custom implementation of transform with custom rotation and translation 
    to ease mean and standard deviation calculations.
*/

struct rotation {
    float sin_roll, cos_roll;
    float sin_pitch, cos_pitch;
    float sin_yaw, cos_yaw;
    float add_count;
    float roll;
    float pitch;
    float yaw;

    rotation(float roll=0, float pitch=0, float yaw=0, float sin_roll=0, float cos_roll=0,
            float sin_pitch=0, float cos_pitch=0, float sin_yaw=0, float cos_yaw = 0 ): roll(roll), pitch(pitch), yaw(yaw),
    sin_roll(sin_roll), cos_roll(cos_roll), sin_pitch(sin_pitch), cos_pitch(cos_pitch), sin_yaw(sin_yaw), cos_yaw(cos_yaw){}

    rotation& operator=(const rotation& a) 
    {
        roll = a.roll;
        pitch = a.pitch;
        yaw = a.yaw;
        add_count = a.add_count;
        return *this;
    }

    void operator+=(const rotation& a) 
    { 
    // Accumulate rotation data by casting it on unit circle for each axis

        sin_roll += sin(a.roll);
        cos_roll += cos(a.roll); 
        sin_pitch += sin(a.pitch);
        cos_pitch += cos(a.pitch);
        sin_yaw += sin(a.yaw);
        cos_yaw += cos(a.yaw);

        add_count++;
    }

    rotation getStDev()
    { 
    //scipy implementation of single pass, circular standard deviation
    //https://github.com/scipy/scipy/blob/41dd89916870ea215f612727ecf42d12ea434657/scipy/stats/_morestats.py#L3599

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

    rotation getMean()
    {
    //scipy implementation of single pass, circular data mean
    //https://github.com/scipy/scipy/blob/41dd89916870ea215f612727ecf42d12ea434657/scipy/stats/_morestats.py#L3546
        rotation ret;
        
        ret.roll = atan2(sin_roll , cos_roll);
        ret.pitch = atan2(sin_pitch , cos_pitch);
        ret.yaw = atan2(sin_yaw , cos_yaw);

        return ret;
    }
};

struct translation {
        float x_sq_sum = 0; // sum of squares are needed for the single pass standard dev algorithm
        float y_sq_sum = 0;
        float z_sq_sum = 0;
        float add_count = 0;
        float x;
        float y;
        float z;

        translation(float x=0, float y=0, float z=0, float x_sq_sum=0, float y_sq_sum=0, float z_sq_sum=0): 
                x(x), y(y), z(z), x_sq_sum(x_sq_sum), y_sq_sum(y_sq_sum), z_sq_sum(z_sq_sum) {}

        translation& operator=(const translation& a) //common
        {
            x = a.x;
            y = a.y;
            z = a.z;
            add_count = a.add_count;
            return *this;
        }

        void operator+=(translation a)
        {
            x += a.x;
            y += a.y;
            z += a.z;

            x_sq_sum += a.x * a.x;
            y_sq_sum += a.y * a.y;
            z_sq_sum += a.z * a.z;

            add_count++;
        }

        translation getMean() 
        {
            translation ret;
            ret.x =  x/add_count; 
            ret.y =  y/add_count; 
            ret.z =  z/add_count; 
            return ret;
        }

        translation getStDev()
        {
        // Single pass standard deviation algorithm from https://www.strchr.com/standard_deviation_in_one_pass
            translation ret;
            translation avg = this->getMean();

            float variance_x = x_sq_sum / add_count - avg.x*avg.x;
            float variance_y = y_sq_sum / add_count - avg.y*avg.y;
            float variance_z = z_sq_sum / add_count - avg.z*avg.z;

            ret.x = sqrt(variance_x);
            ret.y = sqrt(variance_y);
            ret.z = sqrt(variance_z);

            return ret;
        }
};


struct genericT 
{
    translation trans;
    rotation rot;
    float add_count;

    genericT( float x=0, float y=0, float z=0, float roll=0, float pitch=0, float yaw=0, float add_count=0) 
        : add_count(0)
    {
        trans.x = x; 
        trans.y = y; 
        trans.z = z; 
        trans.add_count = add_count; 
        rot.roll = roll; 
        rot.pitch = pitch; 
        rot.yaw = yaw;   
        rot.add_count = add_count; 
    }

    genericT& operator=(const genericT& a) //common
    {
        trans = a.trans;
        rot = a.rot;
        add_count = a.add_count;
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

    // Process rotation and translation seperately since they hold periodic and non-periodic data respectively.
    // Different methods for these stats are already exclusively implemented in those types.
    genericT getMean()
    {
        genericT ret;
        ret.rot = rot.getMean();
        ret.trans = trans.getMean();
        return ret;
    }

    genericT getStDev()
    {
        genericT ret;
        ret.rot = rot.getStDev();
        ret.trans = trans.getStDev();
        return ret;
    }
};

struct statType 
{
    genericT mean;
    genericT dev;
};
