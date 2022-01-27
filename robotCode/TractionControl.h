#include "movement.h"

class TractionControl
{
private:
    TalonPair * __bleft;
    TalonPair * __brght;
    TalonPair * __fleft;
    TalonPair * __frght;
public:
    TractionControl(TalonPair*, TalonPair* , TalonPair*, TalonPair*);
    ~TractionControl();
};

TractionControl::TractionControl(TalonPair* b_left, TalonPair* b_rght , TalonPair* f_left, TalonPair* f_rght)
{
    __bleft = b_left;
    __brght = b_rght;
    __fleft = f_left;
    __frght = f_rght;

}

TractionControl::~TractionControl()
{
    cout<<"Traction Control object successfully destroyed\n";
}
