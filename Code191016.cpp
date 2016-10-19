//----------------------------------------------------------------------------//
////////////////////////////////////////////////////////////////////////////////
//      programme principale VAE
//      date de création    :21/09/2016
//      date de mise à jour :05/10/2016
//      détails             :
//
////////////////////////////////////////////////////////////////////////////////
/******************************** DEFINE PRGM *********************************/
////////////////////////////////////////////////////////////////////////////////
#define MAIN
//#define BOUCLETEST
//#define DEBUG
#define PRGM

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
#define RADIUS  0.192F // wheel size
#define NBPOLES 8    // magnetic pole number
#define DELTA_T 0.091F // speed measurement counting period
#define PI 3.1415926535
#define NBIT 255
#define STEP_VIT 1
#define VITMAXMH 30000 //m/h
#define TEMPSENSI 0.01 //mV/° 
#define DELTAVBAT 0.013 //coef directeur
#define IBATSENSI 0.185  //mV/A

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
/********************* web server section *************************************/
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
/*********************** can bus section  *************************************/
//----------------------------------------------------------------------------//
//determine message ID used to send Gaz ref over can bus
#define _CAN_DEBUG              // used to debug can bus activity
//#define USE_CAN_REF
// uncomment to receive gaz ref over can_bus initialisation du Bus CAN sur
//les broches 30 (rd) et 29(td) for lpc1768 + mbed shield
CAN can_port (p30, p29);
bool bCan_Active=false;

DigitalOut led1(LED1); // initialisation des Leds présentes sur
DigitalOut led2(LED2); // le micro-controleur Mbed*\
DigitalOut led3(LED3); // blink when can message is sent
DigitalOut led4(LED4); // blink when can message is received

InterruptIn Top_Hall(p22);

AnalogIn poignevitesse(p17);
AnalogIn NBat(p18);
AnalogIn NTemp(p19);
AnalogIn NImes(p20);


//----------------------------------------------------------------------------//
/******* timer interrupt for speed measurement each 91ms  *********************/
//----------------------------------------------------------------------------//
Ticker CalculSpeed;
//----------------------------------------------------------------------------//
//******** Timer Interrupt for gaz ref management each 10ms   *****************/
//----------------------------------------------------------------------------//
Ticker RefreshSpeed;
//----------------------------------------------------------------------------//
/************* local function prototypes **************************************/
//----------------------------------------------------------------------------//
void SendVitessePoigneeGaz(void);
float VitesseProgressive(float);
void CalibrationPoignee(void);
void ReceiveDataSpeed(void);
void CompteurFrontUp(void);
void ReadTemperature(void);
void ReadVBatterie(void);
void ReadIBatterie(void);
float LimiteBride(float);
void PrintDataPLD(void);
void ReadDataPLD(void);
void WriteFile(void);
void SetBride(void);
float Arrondi(float);
int ReadFile(void);
void SetPwm(float);

//----------------------------------------------------------------------------//
/************* Globales Variables Main ****************************************/
//----------------------------------------------------------------------------//
var_field_t tab_balise[10];     // une balise est présente dans le squelette
int giCounter=0;                // acces counting
int giHall,giDir,giOverCurrent,giFLTA,giBrake,giConsigne;
float gfVitesse=0;
int VState = 0, VcptProg = 0;
float gfGmin = 0, gfGmax = 0;
int giBride = VITMAXMH;
int gicptTopHall=0;

//----------------------------------------------------------------------------//
/********* main cgi function used to patch data to the web server thread ******/
//----------------------------------------------------------------------------//
void CGI_Function(void) // cgi function that patch web data to empty web page
{
    char ma_chaine4[20]= {}; // needed to form html response

}

//----------------------------------------------------------------------------//
/*********************** CAN BUS SECTION  *************************************/
//----------------------------------------------------------------------------//
void CAN_REC_THREAD(void const *args)
{
    int iCount,iError;
    while (bCan_Active) {
        Thread::wait(100);// wait 100ms
        // code todo
    }
}

////////////////////////////////////////////////////////////////////////////////
/*************************** main programme ***********************************/
////////////////////////////////////////////////////////////////////////////////
int main()
{
    pc.printf(" programme scooter mbed \n\r");

    //variable(s)
    char cChoix=0;
    float fPwmref=0;

    //initialisation
    gicptTopHall=0;
    SetPwm(0);
    Top_Hall.rise(&CompteurFrontUp);
    CalculSpeed.attach(&ReceiveDataSpeed,DELTA_T);
    volatile int vifile = ReadFile();

//----------------------------------------------------------------------------//
//***************************************** web section **********************//
//Init_Web_Server(&CGI_Function); // create and initialize tcp server socket *//
//and pass function pointer to local CGI function*****************************//
//Thread WebThread(Web_Server_Thread);// create and launch web server thread**//
//**** main cgi function used to patch data to the web server thread *********//
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
//****************************** Programme test ******************************//
//----------------------------------------------------------------------------//
#ifdef BOUCLETEST
    while(1) {
        //pc.printf(" gvit = %g \n\r",poignevitesse.read());
        ReadIBatterie();
        wait(0.5);
    }
#endif //BOUCLETEST
//----------------------------------------------------------------------------//
//******************** can bus section initialisation ************************//
//----------------------------------------------------------------------------//
//bCan_Active=true;// needed to lauchn CAN thread
//Thread CanThread(CAN_REC_THREAD);// create and launch can receiver  thread

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
        pc.scanf(" %c",&cChoix);
        switch (cChoix) {
            case 'a':
                RefreshSpeed.detach();
                pc.printf("saisir une Pwm entre 0%% et 100%% =>");
                pc.scanf("%g",&fPwmref);
                SetPwm(fPwmref);
                break;
            case 'b':
                printf("Vitesse = %1.3f m/s\n\r", gfVitesse);
                printf("Vitesse = %1.3f m/h\n\r", gfVitesse*3600);
                break;
            case 'c':
                ReadDataPLD();
                PrintDataPLD();
                break;
            case 'd':
                CalibrationPoignee();
                break;
            case 'e':
                int file = ReadFile();
                if((gfGmin==0 && gfGmax==0) || !file)
                    CalibrationPoignee();
                RefreshSpeed.attach(&SendVitessePoigneeGaz,0.010);
                break;
            case 'f':
                SetBride();
                break;
            case 'g':
                ReadIBatterie();
                break;
            case 'h':
                ReadVBatterie();
                break;
            case 'i':
                ReadTemperature();
                break;
            case 'j':
                ReadIBatterie();
                ReadVBatterie();
                ReadTemperature();
                break;
            case 'q':
                SetPwm(0);
                RefreshSpeed.detach();
                break;
        }
        wait(0.005);
    } // end while
//----------------------------------------------------------------------------//
//************* thread deinit ************************************************//
//----------------------------------------------------------------------------//
    //DeInit_Web_Server();
    //bCan_Active=false;
    //CanThread=false;// close can received thread
    pc.printf(" fin programme scooter mbed \n\r");
}
//----------------------------------------------------------------------------//
#endif //PRGM
////////////////////////////////////////////////////////////////////////////////
/**************************** Fonctions ***************************************/
////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------//
//Permet de définir le rapport cyclique
void SetPwm(float fPwmref)
{
    fPwmref = fPwmref*2.55; // fPwmref * 255(8bits) / 100 (0 à 100)
    MyPLD.write(fPwmref); //write onto pld
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
    gfGmin = poignevitesse.read();
    pc.printf("\nmettre la poignee au maximum et saisir z \n\r");
    do {
        c2=getchar();
    } while(c2!='z');
    gfGmax = poignevitesse.read();
    pc.printf("\ngfGmin => %1.3f & gfGmax => %1.3f\n\r",gfGmin,gfGmax);
    WriteFile();
}
//----------------------------------------------------------------------------//
float VitesseProgressive(float fPwm)
{
    static float sfPwmref = 0;
    if(sfPwmref < fPwm)
        sfPwmref += STEP_VIT;
    else if(sfPwmref >= fPwm)
        sfPwmref = fPwm;
#ifdef DEBUG
    pc.printf("=> %1.3f\n\r",sfPwmref);
#endif //DEBUG
    return sfPwmref;
}
//----------------------------------------------------------------------------//
float LimiteBride(float fPwmref)
{
    float gfBride = giBride;
    fPwmref = fPwmref * (gfBride / VITMAXMH);
    return fPwmref;
}
//----------------------------------------------------------------------------//
//Send the speed to the Altera
void SendVitessePoigneeGaz(void)
{
    ReadDataPLD();
    float fPwmref;
    fPwmref = (poignevitesse.read()-gfGmin)*(NBIT)/(gfGmax-gfGmin);
    if(!giBrake)
        fPwmref = 0;
    fPwmref = LimiteBride(fPwmref);
    fPwmref = VitesseProgressive(fPwmref);
    MyPLD.write(fPwmref);
}
//----------------------------------------------------------------------------//
void SetBride (void)
{
    pc.printf("saisir une valeur de bride => ");
    pc.scanf("%d",&giBride);
    if(giBride > VITMAXMH)
        giBride = VITMAXMH;
    WriteFile();
}
//----------------------------------------------------------------------------//
//Write the calibration data into the file to save it
void WriteFile (void)
{
    FILE *fichier=NULL;
    fichier = fopen("/local/PARA_1A.txt","w");
    fprintf(fichier,"%1.3f %1.3f %d",gfGmin, gfGmax, giBride);
    fclose(fichier);
}
//----------------------------------------------------------------------------//
//Read Calibration Data
int ReadFile(void)
{
    FILE *fichier=NULL;
    fichier = fopen("/local/PARA_1A.txt","r");
    if (fichier != NULL) {
        fscanf(fichier,"%f %f %d",&gfGmin, &gfGmax, &giBride);
        fclose(fichier);
#ifdef DEBUG
        pc.printf("%f %f %d",&gfGmin, &gfGmax, &giBride);
#endif //DEBUG
    } else {
        // On affiche un message d'erreur si on veut
        pc.printf("Impossible d'ouvrir le fichier\n\r");
        return 0;
    }
    return 1;
}
//----------------------------------------------------------------------------//
//compteur Capteur HALL
void CompteurFrontUp(void)
{
    gicptTopHall++;
}
//----------------------------------------------------------------------------//
//arrondi au centième
float Arrondi(float fVal)
{
    fVal = fVal*10;
    int iVal = fVal;
    if ((fVal - iVal) >= 0.5)
        return((fVal + 1)/10);
    else
        return(fVal/10);
}
//----------------------------------------------------------------------------//
void ReadVBatterie(void)
{
    float fval=NBat.read();
    pc.printf("fval(VBat) = %0.2f V\n\r",Arrondi(fval/DELTAVBAT));

}
//----------------------------------------------------------------------------//
void ReadIBatterie(void)
{
    float fval=NImes.read();
    fval = Arrondi((((fval*3.3*1.85)-2.5)/IBATSENSI)*1000);
    if(fval<0)
        fval=0;
    pc.printf("fval(VImes) = %0.2f mA\n\r",fval);

}
//----------------------------------------------------------------------------//
void ReadTemperature(void)
{
    float fval=NTemp.read();
    fval = Arrondi((fval*3.3)/TEMPSENSI);
    pc.printf("fval(VTemps) = %0.2f C\t = %0.2f K\n\r ",fval-273.5,fval);

}
//----------------------------------------------------------------------------//
//Receive the data coming from the Altera
void ReceiveDataSpeed(void)
{
    gfVitesse = ((gicptTopHall * 2 * PI * RADIUS) / (6 * NBPOLES * DELTA_T));
    gicptTopHall = 0;
}
//----------------------------------------------------------------------------//
//Imprime sur le terminal les valeurs du PLD
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
#endif //MAIN
//----------------------------------------------------------------------------//