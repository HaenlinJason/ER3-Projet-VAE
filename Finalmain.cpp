//----------------------------------------------------------------------------//
////////////////////////////////////////////////////////////////////////////////
//      programme principale VAE
//      date de création    :21/09/2016
//      date de mise à jour :20/12/2016
//      détails             :VAE ER3 project for S3 students
//
////////////////////////////////////////////////////////////////////////////////
/******************************** DEFINE PRGM *********************************/
////////////////////////////////////////////////////////////////////////////////
#define MAIN         //all the programme 
//#define BOUCLETEST //disable comentary to try a sigle function 
//#define DEBUG      //disable comentary to add information in the terminal
#define PRGM         //main programme

////////////////////////////////////////////////////////////////////////////////
/******************************** Main ****************************************/
////////////////////////////////////////////////////////////////////////////////
#ifdef MAIN

//----------------------------------------------------------------------------//
/***************************** Library ****************************************/
//----------------------------------------------------------------------------//
//#include "EthernetInterface.h"
#include <stdlib.h>
#include <string.h>
#include "mbed.h"
#include "rtos.h"    // need for main thread sleep
#include "html.h"    // need for html patch working with web server
#include "bloc_io.h"

////////////////////////////////////////////////////////////////////////////////
/******************************** Constantes **********************************/
////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------//
#define RADIUS  0.192F          // wheel size
//----------------------------------------------------------------------------//
#define NBPOLES 8               // magnetic pole number
//----------------------------------------------------------------------------//
#define DELTA_T 0.091F          // speed measurement counting period
//----------------------------------------------------------------------------//
#define PI 3.1415926535         
//----------------------------------------------------------------------------//
#define NBIT 255                //PLD max bits => 8 bytes -> 0 to 255 bits
//----------------------------------------------------------------------------//
#define STEP_VIT 1              //use to increase the speed softly or to
                                //decrease it when the speed is at the max speed
                                //depending of the limit.
//----------------------------------------------------------------------------//
#define VITMAXMH 50000          //meter/hour
                                //define the default limit : 50 000 mean there 
                                //is no limit.
//----------------------------------------------------------------------------//
#define TEMPSENSI 0.01          //mV/degree 
                                //sensor sensitivity for the temperature
//----------------------------------------------------------------------------//
#define DELTAVBAT 0.013         //leading coefficient for the battery tension
//----------------------------------------------------------------------------//
#define IBATSENSI 0.185         //mV/A
                                //sensor sensitivity for the electric current
//----------------------------------------------------------------------------//
#define IMAXBATT  3600000       //mAs
                                //max eletric current for the test
//----------------------------------------------------------------------------//
#define UPDATETIMEBATT 0.1      //s
                                //to attach a ticker which calculates the 
                                //Battery remaining
//----------------------------------------------------------------------------//
#define ECH 10                  //define the number of sample that we need 
                                //before doing the average to get the best 
                                //precision in the current to update 
                                //the Battery remaining
//----------------------------------------------------------------------------//
/************ Ports de communications *****************************************/
//----------------------------------------------------------------------------//
// instantiate object needed to communicate with PLD
// analog input connected to mbed
Bloc_IO MyPLD(p25,p26,p5,p6,p7,p8,p9,p10,p23,p24);
// valid pmw mbed pin
Serial pc(USBTX, USBRX);        // tx, rx
// Top_Hall Pin

//----------------------------------------------------------------------------//
/************ persistent file parameters section ******************************/
//----------------------------------------------------------------------------//
LocalFileSystem local("local"); // Create the local filesystem
// under the name "local"

//----------------------------------------------------------------------------//
/*********************** can bus section  *************************************/
//----------------------------------------------------------------------------//
//determine message ID used to send Gaz ref over can bus
#define _CAN_DEBUG              // used to debug can bus activity
#define USE_CAN_REF
//uncomment to receive gaz ref over can_bus 
//30 (rd) & 29(td) for lpc1768 + mbed shield
CAN can_port (p30, p29);
bool bCan_Active=false;
//----------------------------------------------------------------------------//
/*********************** communications ports  ********************************/
//----------------------------------------------------------------------------//
InterruptIn Top_Hall(p22);
//sensor on the wheel to determine the rotor position 
//----------------------------------------------------------------------------//
AnalogIn poignevitesse(p17);
//Gas handle sensor to manage the speed
//----------------------------------------------------------------------------//
AnalogIn NBat(p18);
//voltage sensor receive in volt
//----------------------------------------------------------------------------//
AnalogIn NTemp(p19);
//temperture sensor receive in volt
//----------------------------------------------------------------------------//
AnalogIn NImes(p20);
//eletric current sensor receive in volt
//----------------------------------------------------------------------------//
/******* timer interrupt for speed measurement each 91ms  *********************/
//----------------------------------------------------------------------------//
Ticker CalculSpeed;
//----------------------------------------------------------------------------//
//******** Timer Interrupt for gaz ref management each 10ms   *****************/
//----------------------------------------------------------------------------//
Ticker RefreshSpeed;
//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//
Ticker RefreshBatterie;
//----------------------------------------------------------------------------//
/************* local function prototypes **************************************/
//----------------------------------------------------------------------------//
void SendVitessePoigneeGaz(void);
//Send the vitesse from the speed handle to the rotor
//this function use the "VitesseProgressive" function which include 
//the Progressive speed and the speed limit and include the brake
//by reading the PLD informations on the "ReadDataPLD" function
//----------------------------------------------------------------------------//
float VitesseProgressive(float);
//this function include the progressive function and the speed limit
//----------------------------------------------------------------------------//
void CalibrationPoignee(void);
//used to calibrate the speed handle
//----------------------------------------------------------------------------//
void ReceiveDataSpeed(void);
//used to read the current speed 
//----------------------------------------------------------------------------//
float ReadTemperature(void);
//receive a float in kelvin(K)
//----------------------------------------------------------------------------//
void CompteurFrontUp(void);
//attach to an interrupt every time there is a rising edge we add one 
//to gicptTopHall and it's used to calculate the speed (ReceiveDataSpeed())
//----------------------------------------------------------------------------//
float ReadVBatterie(void);
//receive the battery voltage as a float in volt(V)
//----------------------------------------------------------------------------//
float ReadIBatterie(void);
//receive the battery electric current as a float in volt(V) 
//----------------------------------------------------------------------------//
void UpdateBatterie(void);
//battery duration
//----------------------------------------------------------------------------//
void PrintDataPLD(void);
//receive the data from the PLD (Programmable Logic Device) 
//----------------------------------------------------------------------------//
void CGI_Function(void);
//function that patch web data to an empty web page
//----------------------------------------------------------------------------//
void ReadDataPLD(void);
//read the data from Programmable Logic Device on Altera(Quartus)
//----------------------------------------------------------------------------//
void WriteFile(void);
//Write the data in a file
//----------------------------------------------------------------------------//
void Setlimit(void);
//set the speed limit up to 50 000m/h (50km/h ou 13,89m/s)
//----------------------------------------------------------------------------//
float Arrondi(float);
//Rounded to the hundredth 
//----------------------------------------------------------------------------//
int ReadFile(void);
//read in the file, the function send a '0' if there are an error '1' if nothing
//----------------------------------------------------------------------------//
void SetPwm(float);
//set the pwm from 0% to 100% (respectively 0 to 255)
//----------------------------------------------------------------------------//
/************* Globales Variables  ********************************************/
//----------------------------------------------------------------------------//
//const uint16_t SIGNAL_ID = 0x42; //needed if there are more then one CAN to 
                                   //know which is the right one
var_field_t tab_balise[10];    
int giCounter=0;                // acces counting
int iCounter=0;
int giHall,giDir,giOverCurrent,giFLTA,giBrake,giConsigne;
float gfVitesse=0;
int VState = 0, VcptProg = 0;
float gfGmin = 0, gfGmax = 0;
int giBride = VITMAXMH;
int gicptTopHall=0;
int itab[ECH];
float gfIJaugeBatt = IMAXBATT; //electric current max of the battery
float gfCanPoignee; 
CANMessage msg;

//----------------------------------------------------------------------------//
/*********************** CAN BUS SECTION  *************************************/
//----------------------------------------------------------------------------//
void CAN_REC_THREAD(void const *args)
{
    //int iCount,iError;
    while (bCan_Active) {
        Thread::wait(100);// wait 100ms
        if(can_port.read(msg)) {
            gfCanPoignee = *reinterpret_cast<float*>(msg.data);
            //char data reinterpret into float data
            //pc.printf("  signal  = %1.3f\r\n", gfCanPoignee);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*************************** main programme ***********************************/
////////////////////////////////////////////////////////////////////////////////
int main()
{
    pc.printf(" programme scooter mbed \n\r");
    //variables
    char cChoix=0;   //state machine
    float fPwmref=0; //motor pwm at 0 for the manuel use only 
                     //pwm bettween 0 and 255 
    //initialisation
    gicptTopHall=0;
    SetPwm(0);       //set the pwm of the motor at 0 (Does not rotate)
//----------------------------------------------------------------------------//
//if the file isn't created we create it and make the calibration
    if(!ReadFile())
        CalibrationPoignee();
//----------------------------------------------------------------------------//
    Top_Hall.rise(&CompteurFrontUp);//when the sensor position change 
                                    //the function is called 
    CalculSpeed.attach(&ReceiveDataSpeed,DELTA_T);
                                    //estimate the speed 
    RefreshBatterie.attach(&UpdateBatterie,UPDATETIMEBATT);
                                    //manage the battery duration
    RefreshSpeed.attach(&SendVitessePoigneeGaz,0.010);
                                    //refresh the speed compared to the 
                                    //speed handle
//----------------------------------------------------------------------------//
//**************************** web section ***********************************//
    Init_Web_Server(&CGI_Function); // create and initialize tcp server socket
//and pass function pointer to local CGI function
    Thread WebThread(Web_Server_Thread);// create and launch web server thread
    Gen_HtmlCode_From_File("/local/pagecgi2.htm",tab_balise,7);// read and
//localise ^VARDEF[X] tag in empty html file
//----------------------------------------------------------------------------//
//******************** can bus section initialisation ************************//
//----------------------------------------------------------------------------//
    bCan_Active=true;// needed to launch CAN thread
    Thread CanThread(CAN_REC_THREAD);// create and launch can receiver thread
//----------------------------------------------------------------------------//
//****************************** Programme test ******************************//
//----------------------------------------------------------------------------//
#ifdef BOUCLETEST
    while(1) {
        //pc.printf(" gvit = %g \n\r",poignevitesse.read());
        //pc.printf("Batterie restante: %d Ah\n\r",
        //Arrondi(int(gfIJaugeBatt)));
        //pc.printf("fval(VBat) = %0.2f V\n\r",fval);
        //pc.printf("fval(VTemps) = %0.2f C\t = %0.2f K\n\r ",fval-273.5,fval);
        //pc.printf("fval(VImes) = %0.2f mA\n\r",fval);
        wait(0.5);
    }
#endif //BOUCLETEST
#ifdef PRGM
//----------------------------------------------------------------------------//
//********************* MENU *************************************************//
//----------------------------------------------------------------------------//
    while(cChoix!='q' and cChoix!='Q') {
        pc.printf(" \n\rveuillez saisir un choix parmi la liste proposee: \n\r");
        pc.printf(" a: Mode Manuel: saisie consigne pwm \n\r");
        pc.printf(" b: Vitesse de la roue\n\r");
        pc.printf(" c: Lecture du PLD\n\r");
        pc.printf(" d: Calibration de la Poignee\n\r");
        pc.printf(" e: Mode Auto: Vitesse a l aide de la poignee de gaz\n\r");
        pc.printf(" f: Definir le bridage\n\r");
        pc.printf(" g: Courant de la batterie\n\r");
        pc.printf(" h: Tension de la batterie\n\r");
        pc.printf(" i: Temperature\n\r");
        pc.printf(" j: Donnees capteurs\n\r");
        pc.printf(" k: Jauge de la batterie\n\r");
        pc.printf(" l: reset Jauge batterie\n\r");
        pc.printf(" q: quitter\n\r");

        // multithreading : main thread need
        //to sleep in order to allow web response
        while (pc.readable()==0) {// determine if char availabler
            Thread::wait(10);   // wait 10 until char available on serial input
        }
        //--------------------------------------------------------------------//
        /************* end of main thread sleep  ****************/
        //--------------------------------------------------------------------//
//----------------------------------------------------------------------------//
//******************** State Machine *****************************************//
//----------------------------------------------------------------------------//
        pc.scanf("%c",&cChoix);
        switch (cChoix) {
            case 'a'://manual mode (0 to 100%)
                RefreshSpeed.detach();
                pc.printf("saisir une Pwm entre 0%% et 100%% =>");
                pc.scanf("%g",&fPwmref);
                SetPwm(fPwmref);
                break;
            case 'b'://read the speed 
                printf("Vitesse = %1.3f m/s\n\r", gfVitesse);
                printf("Vitesse = %1.3f m/h\n\r", gfVitesse*3600);
                break;
            case 'c'://read and print the PLD data in the terminal
                ReadDataPLD();
                Printeletric currentPLD();
                break;
            case 'd'://speed handle calibration
                CalibrationPoignee();
                break;
            case 'e'://automatique speed with the handle
                int file = ReadFile();//verify if the file is correctly write
                if((gfGmin==0 && gfGmax==0) || !file)
                    CalibrationPoignee();
                RefreshSpeed.attach(&SendVitessePoigneeGaz,0.010);
                break;
            case 'f'://set the speed limite
                Setlimit();
                break;
            case 'g'://battery electric current
                pc.printf("fval(VImes) = %0.2f mA\n\r",ReadIBatterie());
                break;
            case 'h'://battery voltage
                pc.printf("fval(VBat) = %0.2f V\n\r",ReadVBatterie());
                break;
            case 'i'://controller temperature
                pc.printf("fval(VTemps) = %0.2f C\t = %0.2f K\n\r ",
                          ReadTemperature()-273.5,ReadTemperature());
                break;
            case 'j'://case g, h & i
                pc.printf("fval(VImes)  = %0.2f mA\n\r",ReadVBatterie());
                pc.printf("fval(VBat) = %0.2f V\n\r",ReadVBatterie());
                pc.printf("fval(VTemps) = %0.2f K\t = %0.2f C\n\r ",
                          ReadTemperature()+273.15,ReadTemperature());
                break;
            case 'k'://Battery remaining
                pc.printf("Batterie restante: %0.2f %%\n\r",(gfIJaugeBatt/IMAXBATT)*100);
                break;
            case 'l'://recharge the battery
                gfIJaugeBatt = IMAXBATT;
                RefreshBatterie.attach(&UpdateBatterie,UPDATETIMEBATT);
                pc.printf("\n\rBatterie recharge!!\n\r");
                break;
            case 'q'://stop the programme
                SetPwm(0);
                RefreshSpeed.detach();
                RefreshBatterie.detach();
                WriteFile();
                break;
        }
        wait(0.005);
    } // end while
//----------------------------------------------------------------------------//
//************* thread deinit ************************************************//
//----------------------------------------------------------------------------//
    DeInit_Web_Server();
    bCan_Active=false;
    CanThread=false;// close can received thread
    pc.printf(" fin programme scooter mbed \n\r");
}
//----------------------------------------------------------------------------//
#endif //PRGM
////////////////////////////////////////////////////////////////////////////////
/**************************** Fonctions ***************************************/
////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------//
//define the motor cyclic ratio 
void SetPwm(float fPwmref)
{
    fPwmref = fPwmref*2.55; // fPwmref * 255(8bits) / 100 (0 à 100)
    MyPLD.write(fPwmref); //write in the pld
}
//----------------------------------------------------------------------------//
//calibre la poignee pour bien definir le scale
void CalibrationPoignee(void)
{
    char c1,c2;
    pc.printf("\nmettre la poignee au minimum et saisir z \n\r");
    do {
        c1=getchar();
    } while(c1!='z');
    gfGmin = gfCanPoignee;//poignevitesse.read();
                          //used before the CAN
    pc.printf("\nmettre la poignee au maximum et saisir z \n\r");
    do {
        c2=getchar();
    } while(c2!='z');
    gfGmax = gfCanPoignee;//poignevitesse.read();
                          //used before the CAN
    pc.printf("\ngfGmin => %1.3f & gfGmax => %1.3f\n\r",gfGmin,gfGmax);
    WriteFile();
}
//----------------------------------------------------------------------------//
float VitesseProgressive(float fPwm)
{
    //static is set at the first execution of the function and is never
    //Initialized again
    static float sfPwmref = 0;
    //bride--------------------------------------------------//
    if((gfVitesse*3600) > giBride) { // gfvitesse in m/s
        if (sfPwmref>0)
            sfPwmref -= STEP_VIT;
    }
    //end bride----------------------------------------------//
    //acceleration-------------------------------------------//
    else {
        if(sfPwmref < fPwm)
            sfPwmref += STEP_VIT;
        else
            sfPwmref = fPwm;
    }
    //end acceleration---------------------------------------//
    return sfPwmref;
}
//----------------------------------------------------------------------------//
//Send the speed to the Altera
void SendVitessePoigneeGaz(void)// ticker launch
{
    ReadDataPLD();
    float fPwmref;
    fPwmref = (gfCanPoignee-gfGmin)*(NBIT)/(gfGmax-gfGmin);
    //fPwmref = (poignevitesse.read()-gfGmin)*(NBIT)/(gfGmax-gfGmin);
    //used before the CAN
    if(!giBrake)
        fPwmref = 0;
    else
        fPwmref = VitesseProgressive(fPwmref);
    MyPLD.write(fPwmref);
}
//----------------------------------------------------------------------------//
void Setlimit (void)
{
    pc.printf("saisir une valeur de bridage en m/h => ");
    pc.scanf("%d",&giBride);// read giBride in m/h
    if(giBride > VITMAXMH)
        giBride = VITMAXMH;
    WriteFile();
}
//----------------------------------------------------------------------------//
//Write the calibration eletric current into the file to save it
void WriteFile (void)
{
    FILE *fichier=NULL;
    fichier = fopen("/local/PARA_1A.txt","w");
    fprintf(fichier,"%1.3f %1.3f %d %f",gfGmin, gfGmax, giBride,gfIJaugeBatt);
    fclose(fichier);
}
//----------------------------------------------------------------------------//
//Read Calibration eletric current
int ReadFile(void)
{
    FILE *fichier=NULL;
    fichier = fopen("/local/PARA_1A.txt","r");
    if (fichier != NULL) {
        fscanf(fichier,"%f %f %d %f",&gfGmin, &gfGmax, &giBride, &gfIJaugeBatt);
        fclose(fichier);
#ifdef DEBUG
        pc.printf("%f %f %d",&gfGmin, &gfGmax, &giBride);
#endif //DEBUG
    } else {
        // error message if we can't open the file
        pc.printf("Impossible d'ouvrir le fichier\n\r");
        return 0;
    }
    return 1;
}
//----------------------------------------------------------------------------//
//counter when the rotor position as changed 
void CompteurFrontUp(void)//interruption
{
    gicptTopHall++;
}
//----------------------------------------------------------------------------//
//Rounded to the hundredth
float Arrondi(float fVal)
{
    fVal = fVal*100;
    int iVal = fVal;
    if ((fVal - iVal) >= 0.5)
        return((fVal + 1)/100);
    else
        return(fVal/100);
}
//----------------------------------------------------------------------------//
float ReadVBatterie(void)
{
    float fval=NBat.read();
    fval=Arrondi(fval/DELTAVBAT);
    return fval;
}
//----------------------------------------------------------------------------//
//Update power battery every 1s
void UpdateBatterie(void)
{
    static int icpt = 0;
    icpt++;
    static float fIconsmoy=0;
    if(icpt <= ECH)
        fIconsmoy = fIconsmoy + 60000;
    if(icpt == ECH) {
        fIconsmoy = fIconsmoy/ECH;
        gfIJaugeBatt = gfIJaugeBatt-fIconsmoy;
        if(gfIJaugeBatt <= 0)
            gfIJaugeBatt = 0;
        fIconsmoy=0;
        icpt=0;
    }
}
//----------------------------------------------------------------------------//
float ReadIBatterie(void)
{
    float fval=NImes.read();
    fval = Arrondi((((fval*3.3*1.85)-2.5)/IBATSENSI)*1000);
    if(fval<0)
        fval=0;
    return fval;
}
//----------------------------------------------------------------------------//
float ReadTemperature(void)
{
    float fval=NTemp.read();
    fval = Arrondi((fval*3.3)/TEMPSENSI);
    return (fval-273.5);
}
//----------------------------------------------------------------------------//
//Receive the data coming from the PLD
void ReceiveDataSpeed(void)// ticker launch
{
    // gfvitesse in m/s
    gfVitesse = ((gicptTopHall * 2 * PI * RADIUS) / (6 * NBPOLES * DELTA_T));
    gicptTopHall = 0;
}
//----------------------------------------------------------------------------//
//print in the terminal the PLD values
void PrintDataPLD(void)
{
    pc.printf("\nSecteur Hall = %d\n\r",giHall);
    if(!giDir)
        pc.printf("Direction = Marche arriere\n\r");
    else
        pc.printf("Direction = Marche avant\n\r");
    if(!giOverCurrent)
        pc.printf("OverCurrent = OUI\n\r");
    else
        pc.printf("OverCurrent = NON\n\r");
    if(!giFLTA)
        pc.printf("FLTA = Pas d'erreur convertisseur\n\r");
    else
        pc.printf("FLTA = Erreur convertisseur\n\r");
    if(!giBrake)
        pc.printf("Frein = OUI\n\r");
    else
        pc.printf("Frein = NON \n\r");

}
//----------------------------------------------------------------------------//
//On fait des masks pour s'éparer les valeurs.
void ReadDataPLD(void)
{
    giHall = MyPLD.read()&7;                //00000111
    giDir  = (MyPLD.read()&8)>>3;           //00001000
    giFLTA  = (MyPLD.read()&16)>>4;         //00010000
    giBrake = (MyPLD.read()&32)>>5;         //00100000
    giOverCurrent = (MyPLD.read()&64)>>6;   //01000000
}
//----------------------------------------------------------------------------//
/********************* web server section *************************************/
//----------------------------------------------------------------------------//
/********* main cgi function used to patch data to the web server thread ******/
void CGI_Function(void) // cgi function that patch web data to empty web page
{
    ReadDataPLD();
    char ma_chaine4[20]= {}; // needed to form html response

    sprintf (ma_chaine4,"%f",ReadVBatterie());// convert speed as ascii string
    Html_Patch (tab_balise,0,ma_chaine4);// patch first label with dyn.string

    sprintf (ma_chaine4,"%f",ReadIBatterie());
    Html_Patch (tab_balise,1,ma_chaine4);

    sprintf (ma_chaine4,"%f",ReadTemperature());
    Html_Patch (tab_balise,2,ma_chaine4);

    sprintf (ma_chaine4,"%d",int(gfVitesse*3600));//m/h
    Html_Patch (tab_balise,3,ma_chaine4);

    sprintf (ma_chaine4,"%d",giHall);
    Html_Patch (tab_balise,4,ma_chaine4);

    sprintf (ma_chaine4,"%d",giBride);
    Html_Patch (tab_balise,5,ma_chaine4);

    sprintf (ma_chaine4,"%f",Arrondi((gfIJaugeBatt/IMAXBATT)*100));
    Html_Patch (tab_balise,6,ma_chaine4);
}
//----------------------------------------------------------------------------//
#endif //MAIN
//----------------------------------------------------------------------------//