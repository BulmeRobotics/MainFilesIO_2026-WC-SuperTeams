#include "Vcameras.h"
#include <UserInterface.h>

//---------------------------------------------------------------------------------------------------------
// Initialization
//---------------------------------------------------------------------------------------------------------

ErrorCodes Vcameras::Init(Ejector* ejector, Mapping* mapper, Driving* robot, UserInterface* ui, Drivetrain* drivetrain){
    _ejector = ejector;
    _mapper = mapper;
    _robot = robot;
    _ui = ui;
    _drivetrain = drivetrain;

    // Set Alert Pins as Input
    pinMode(CAMERA_PIN_INT, INPUT);

    //Set Enabled Pins as Input
    pinMode(CAMERAL_PIN_EN,INPUT);
    pinMode(CAMERAR_PIN_EN,INPUT);

    //Start Communication
    _cam->begin(CAM_BAUD);

    if(_debug_ifc != nullptr) _debug_ifc->println("Start Cam INIT");

    String str;

    uint8_t repeats = CAM_TIMEOUT_INIT / CAM_TIMEOUT;
    for (uint8_t i = 0; i < repeats; i++)
    {
        _cam->print("<I>");
        str = Recieve(CAM_TIMEOUT);
        _connected = (str.indexOf("OK") != -1) ? true : false;
        if (str.indexOf("OK") != -1) break;
    }
    
    if(!_connected) return ErrorCodes::no_connection;
    return ErrorCodes::OK;  
}

//---------------------------------------------------------------------------------------------------------
// Recieve UART
//---------------------------------------------------------------------------------------------------------

String Vcameras::Recieve(uint32_t waittime){
    String str = "";
    uint32_t startTime = millis();

    do {
        while (_cam->available()){  //Is new Character available?
            char c = _cam->read();  //Get char from UART
            if (c == '<') { //Msg start
                str = ""; 
            } else if (c == '>') {  //Msg end
                if(_debug_ifc != nullptr) _debug_ifc->println("Rec: " + str);
                return str;
            } else { 
                str += c;
            }
        }
        if(waittime != 0) delay(1);
    } while (millis() - startTime < waittime);

    return " "; //Timeout
}

bool Vcameras::TryReceivePacketNonBlocking(){
    while (_cam->available()) {
        char c = _cam->read();
        if (c == '<') {
            _rxAsync = "";
        } else if (c == '>') {
            if(_debug_ifc != nullptr) _debug_ifc->println("Rec(NB): " + _rxAsync);
            return true;
        } else {
            _rxAsync += c;
        }
    }
    return false;
}

ErrorCodes Vcameras::EnableNonBlockingStep(){
    if (!_pending) return ErrorCodes::OK;

    if (TryReceivePacketNonBlocking()) {
        String packet = _rxAsync;
        _pending = false;
        _rxAsync = "";
        if (packet.indexOf("OK") != -1) {
            _enabled = _enTarget;
            if(_enabled == false) { _LeftEnabled = false; _RightEnabled = false; }
            return ErrorCodes::OK;
        }
        //_ui->ShowPopup("cams enable error", ErrorCodes::ERROR);
        return ErrorCodes::invalid;
    }

    if ((millis() - _enStart) > CAM_TIMEOUT) {
        _pending = false;
        _ui->ShowPopup("cams enable timeout", ErrorCodes::warning, 2);
        return ErrorCodes::TIMEOUT;
    }

    return ErrorCodes::NO_NEW_DATA;
}

//---------------------------------------------------------------------------------------------------------
// Enable
//---------------------------------------------------------------------------------------------------------

ErrorCodes Vcameras::Enable(bool en, bool blocking){
    if(!_connected) return ErrorCodes::no_connection;   //Check for connection first
    if(en && _victimFound) return ErrorCodes::OK;

    if(_debug_ifc != nullptr){
        _debug_ifc->print(en ? "En" : "Dis");
        _debug_ifc->println(" Cams");
    }

    if (_pending) {
        if (_enTarget != en) {
            // Command changed while waiting -> restart request with latest target.
            _pending = false;
        } else {
            ErrorCodes step = EnableNonBlockingStep();
            if (!blocking || step != ErrorCodes::NO_NEW_DATA) return step;
        }
    }

    if(_enabled == en) return ErrorCodes::OK;

    //Send command
    const char* cmd = en ? "<E>" : "<D>";
    _cam->print(cmd);

    // Prepare async wait state
    _pending = true;
    _enTarget = en;
    _enStart = millis();

    if(!blocking) return ErrorCodes::NO_NEW_DATA;

    // Blocking mode: keep stepping until done or timeout
    while (true) {
        ErrorCodes step = EnableNonBlockingStep();
        if (step == ErrorCodes::NO_NEW_DATA) {
            delay(1);
            continue;
        }
        return step;
    }
}

//---------------------------------------------------------------------------------------------------------
// Handle Reset
//---------------------------------------------------------------------------------------------------------

bool Vcameras::ResetCam(){
    _timeFound = millis();
    _allowEN = false;
    if(!_victimFound) Enable(false, false);
    _victimFound = true;
    return true;
}

ErrorCodes Vcameras::HandleReset(){
    if(!_victimFound || !_allowEN) return ErrorCodes::OK;
    if(_timeFound + DEACT_TIME_VICTIM < millis()){
        _victimFound = false;
        //Enable cams
        Enable(true, false);
        return ErrorCodes::OK;
    }
    return ErrorCodes::disabled;
}

//---------------------------------------------------------------------------------------------------------
// Update
//---------------------------------------------------------------------------------------------------------

ErrorCodes Vcameras::Update(bool onRed, bool onRamp, bool isDelivering, bool allowPickup){
    if(!_connected) {if(_debug_ifc!=nullptr) _debug_ifc->println("Cams no connection");return ErrorCodes::no_connection;}

    if(onRamp != _oldOnRamp){
        if(onRamp && !_oldOnRamp) {
            ResetCam();
            _ui->ShowPopup("Ramp - disable Cam",ErrorCodes::info);
        }
        else _ui->ShowPopup("Ramp end - enable Cam", ErrorCodes::info,3);
        _oldOnRamp = onRamp;
    }

    if(onRamp) return ErrorCodes::ramp;

    // Progress pending async enable commands for both cameras each cycle.
    EnableNonBlockingStep();
    EnableNonBlockingStep();

    if(HandleReset() == ErrorCodes::disabled) {if(_debug_ifc!=nullptr) _debug_ifc->println("Cams Disabled");return ErrorCodes::disabled;}

    if(_oldRed && !onRed) {
        Enable(true, false);
    } else if (!_oldRed && onRed){
        Enable(false, false);
    }
    _oldRed = onRed;
    if(onRed) return ErrorCodes::OK;

    if(!_enabled) { if(_debug_ifc!=nullptr) _debug_ifc->println("Cams Disabled"); return ErrorCodes::disabled;}

    //check if cameras are enabled
    _LeftEnabled = digitalRead(CAMERAL_PIN_EN);
    _RightEnabled = digitalRead(CAMERAR_PIN_EN);

    ReadAlertPin();

    //Continue when no camera is reporting
    if(!_Alert) return ErrorCodes::OK;

    String str = "";
    ErrorCodes side;

    //Wait for new Data
    str = Recieve();
    if(str[0] == ' ') return ErrorCodes::OK;    //Is data valid?

    //Determine victim side
    if(str[0] == 'L') side = ErrorCodes::left;
    else if(str[0] == 'R') side = ErrorCodes::right;
    else return ErrorCodes::invalid;

    // Robot already holds an order and is not delivering: ignore the sighting entirely
    // (rules: only one order at a time; the target will be picked up again in EXPLORE).
    if (!isDelivering && !allowPickup) {
        Recieve(200);	// drain the pending type packet so it is not misread as a side packet later
        return ErrorCodes::disabled;
    }

    //Send stop command
    _drivetrain->Stop();    //Stop robot

    str = Recieve(5000);
    if (str.length() < 2) return ErrorCodes::TIMEOUT;	// type packet timed out / garbage — abort gracefully

    //Determine Victim Type (cameras send '0'..'4' = the five dishes, SUM -2..+2)
    char victim = str[1];

    if (isDelivering) {
        // Rules delivery: blink LED continuously for 3 s while stationary, then place the kit (<=15 cm)
        _ui->Signal(ErrorCodes::LED, 500, 500, 3);
        _ejector->Eject(side, 1);
        _hasDelivered = true;
        ResetCam();	// quiesce the camera so the same target cannot trigger a second kit
        return ErrorCodes::OK;
    }

    //Mapping call
    ErrorCodes err = _mapper->SetVictim();
    if(err != ErrorCodes::OK) {
        if(err == ErrorCodes::already_found) _ui->ShowPopup("Victim alr found",ErrorCodes::warning);
        return err;
    }

    // Rules pickup: the order is only taken if the robot stops at the target for
    // at least 5 s and THEN blinks the SUM count (dish '0'..'4' -> 1..5 blinks).
    uint32_t ts_pickupWait = millis();
    while (millis() - ts_pickupWait < 5000) {
        _ui->Update();
        delay(50);
    }

    uint8_t blinkCount = (victim >= '0' && victim <= '4') ? (uint8_t)(victim - '0') + 1 : 1;
    _ui->Signal(ErrorCodes::LED, 500, 500, blinkCount);

    //Signal Victim
    char buffer[29];
    sprintf(buffer,"ORDER: %c, Side: %c", victim, ((side == ErrorCodes::left) ? 'L' : 'R'));
    _ui->ShowPopup(buffer, ErrorCodes::info, 5);
    _ui->Update();

    _robot->OnVictimDetected();

    ResetCam();	// quiesce the camera so the same target cannot re-trigger immediately
    return ErrorCodes::OK;
}