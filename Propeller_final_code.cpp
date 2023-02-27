#include <stdio.h> // Recommended over iostream for saving space
#include <propeller.h> // Propeller-specific functions
#include <simpletools.h>
#include <time.h>
#include "servo.h"

#define ut1 12 //left ultrasonic trigger
#define ue1 13 //left ultrasonic echo
#define ut2 10 // center ultrasonic tigger
#define ue2 11 //center ultrasonic echo
#define lm 14 //left motor signal line
#define rm 15 //right motor signal line
#define p1 45 //push 1
#define p2 14 //push2 for short push
#define p3 44 //push 3
#define left 14
#define right 17
int inter();
int linloc();
void follower();
int cc();
int cl();
void push();
void puss();
void L90();
void R90();
void object(void *parc);
void isec(void *para);
void blink(void *parb);
void brake();
void spush();
void party();
unsigned int stack1[40 + 25]; // Stack vars for other cog
unsigned int stack2[40 + 25];
unsigned int stack3[40 + 25];
static volatile int cog, cog2, cog3, intno;

int main()
{
int c=0;
int l=0;
int r=0;
int f=0;
int dc;
int dl;
while(1)
{
int sum=inter();
if (sum < 18000 && sum > 2500)
{
int loc=linloc();
if (loc == 2 || loc == 3) //if true, robot moves straight
{
pulse_out(lm,2300);
pulse_out(rm,400);
pause(5);
}
else if (loc == 0 || loc == 1) //if true, robot moves left
{
//pulse_out(lm,1350);
pulse_out(rm,400);
pause(5);
l++;
}
else if (loc == 4 || loc == 5) //if true, robot moves right
{
pulse_out(lm,2300);
//pulse_out(rm,1350);
pause(5);
r++;
}
}
else if(c==3)
{
cog=cogstart(&isec, NULL, stack2, sizeof(stack2));
////////////////////////
push();
dc=cc();
if(dc<45)
intno=2;
else if(dc>45 && dc<70)
intno=3;
else
intno=5;
cog2=cogstart(&blink, NULL, stack1, sizeof(stack1));
spush();
dl = cl();
L90();
pause(25);
//spush();
follower();
//B1
if(dl<40)
{
cog3=cogstart(&object, NULL, stack3, sizeof(stack3));
}
spush();
push();
R90();
for(int bint=0;bint<3;bint++)
{
spush();
follower();
spush();
pause(100);
dl=cl();
if(dl<20)
{
cog3=cogstart(&object, NULL, stack3, sizeof(stack3));
}
}
//B4
spush();
spush();
R90();
push();
follower();
pause(50);
push();
follower();
push();
//A4
pause(25);
dc=cc();
if(dc<20)
{
cog3=cogstart(&object, NULL, stack3, sizeof(stack3));
}
spush();
R90();
for(int aint=0;aint<3;aint++)
{
spush();
follower();
spush();
pause(100);
brake();
dl=cl();
if(dl<20)
{
cog3=cogstart(&object, NULL, stack3, sizeof(stack3));
}
}
//A1
puss();
R90();
push();
pause(25);
follower();
push();
follower();
//B1
push();
R90();
for(int lastrun=0;lastrun<4;lastrun++)
{
follower();
spush();
}
//B5
dl=cl();
if(dl<15)
{
cog3=cogstart(&object, NULL, stack3, sizeof(stack3));
}
pause(400);
for(int x=0;x<=90;x++)
{
pulse_out(rm,400);
pulse_out(lm,2300);
pause(5);
}
party();
break;
}
else if(c==2)
{
cog=cogstart(&isec, NULL, stack2, sizeof(stack2));
c++;
push();
}
else if(c==1 && f==0) //special intersection ignorance case
{
if(l>r)
{
for(int x=0;x<=23;x++)
{
pulse_out(rm,400);
pause(5);
}
f=1;
}
else
{
for(int x=0;x<=23;x++)
{
pulse_out(lm,2300);
pause(5);
}
f=1;
}
c++;
}
else if(c==0)
{
c++;
pause(10);
}
}
}




//------------functions-------------------
int inter()//returns threshold for intersection detection
{
int pins[6]={8, 7, 6, 5, 4, 3};
for(int i=0;i<6;i++)
{
set_direction(pins[i],1);
}
int times[6];
for(int i=0;i<6;i++)
{
high(pins[i]);
pause(1);
set_direction(pins[i],0);
times[i]=rc_time(pins[i],1);
}
int s = 0;
for (int i = 0; i < 6; i++)
{
s += times[i];
}
return s;
}


int linloc()// returns the location of the black line over the sensor array
{
int pins[6]={8, 7, 6, 5, 4, 3};
for(int i=0;i<6;i++)
{
set_direction(pins[i],1);
}
int times[6];
for(int i=0;i<6;i++)
{
high(pins[i]);
pause(1);
set_direction(pins[i],0);
times[i]=rc_time(pins[i],1);
}
int bleh = 0;
int maxind;
for (int i = 0; i < 6; i++)
{
if (times[i] > bleh)
{
bleh = times[i];

maxind = i;
}
}
return maxind;
}

void follower()// follows black line until an intersection is detected
{
int sum=0;
sum=inter();
while(sum < 18500 && sum > 2500)
{
int loc=linloc();
if (loc == 2 || loc == 3) //if true, robot moves straight
{
pulse_out(lm,2300);
pulse_out(rm,400);
pause(5);
}
else if (loc == 0 || loc == 1) //if true, robot moves left
{
//pulse_out(lm,1350);
pulse_out(rm,400);
pause(5);
}
else if (loc == 4 || loc == 5) //if true, robot moves right
{
pulse_out(lm,2300);
//pulse_out(rm,1350);
pause(5);
}
sum=inter();
}
cog=cogstart(&isec, NULL, stack2, sizeof(stack2));
}
//------------------ultrasonic----------------
int cl() // returns distance of nearest object in front of left sensor
{
brake();
//push();
low(ut1);
pause(2);
high(ut1);
pause(10);
low(ut1);
int duration1 = pulse_in(ue1, 1);
int dl = (duration1 * 0.0343) / 2;
return dl;
}
int cc() // returns distance of nearest object in front of center sensor
{ low(ut2);
pause(2);
high(ut2);
pause(10);
low(ut2);
int duration2 = pulse_in(ue2, 1);
int dc = (duration2 * 0.0343) / 2;
return dc;
}
void push() // a linear push
{
for(int x=0;x<=p1;x++)
{
pulse_out(rm,400);
pulse_out(lm,2300);
pause(5);
}
}
void puss() // a linear push
{
for(int x=0;x<=p3;x++)
{
pulse_out(rm,400);
pulse_out(lm,2300);
pause(5);
}
}
void L90() // turn 90 degrees left
{ for(int x=0;x<=left;x++)
{
pulse_out(rm,900);
pulse_out(lm,900);
pause(30);
}
}

void R90() // turn 90 degrees right
{ for(int x=0;x<=right;x++)
{
pulse_out(rm,2000);
pulse_out(lm,2000);
pause(30);
}
}
void brake() // single brake pulse
{
pulse_out(rm,1350);
pulse_out(lm,1350);
}
void object(void *parc) //led blink for object indication
{
//pulse_out(rm,1350);
//pulse_out(lm,1350);
pause(25);
high(2);
pause(500);
low(2);
cogstop(cog3);
}
void isec(void *para) // led blink for intersection indication
{
high(1);
pause(500);
low(1);
cogstop(cog);
}
void blink(void *parb) // led blink for obstacle location indication
{
for(int pav=0; pav<intno; pav++)
{
high(0);
pause(300);
low(0);
pause(300);
}
cogstop(cog2);
}
void party() // random blink sequence at the end
{
high(2);
pause(500);
low(2);
high(1);
pause(500);
low(1);
high(2);
pause(500);
low(2);
high(1);
pause(500);
low(1);
high(2);
pause(500);
low(2);
high(1);
pause(500);
low(1);
}

void spush() // a linear push
{
for(int x=0;x<=p2;x++)
{
pulse_out(rm,400);
pulse_out(lm,2300);
pause(5);
}
}
