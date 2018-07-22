#ifndef __ETHERCAT_ELMO_H
#define __ETHERCAT_ELMO_H

#include <cstdio>
#include <iostream>
#include <cstdint>
#include <vector>
#include <array>
#include <unistd.h>
#include <termios.h>
#include <rtdk.h>

#include "ecrt.h"


//#define hz  2000
#define hz  2000
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / hz)

using namespace std;

namespace EtherCAT_Elmo
{

#define ELMO_GOLD_PRODUCT_CODE  0x00030924
#define ELMO_GOLD_VENDER_ID     0x0000009a

#define ELMO_PDO_ENTRY_SIZE 15

extern ec_pdo_entry_info_t elmo_gold_pdo_entries[];

enum PDOS
{
    TargetPosition,
    TargetVelocity,
    TargetTorque,
    MaxTorque,
    ControlWord,
    ModeOfOperation,

    PositionActualValue,
    PositionFollowingErrrorValue,
    TorqueActualValue,
    StatusWord,
    ModeOfOperationDisplay,

    VelocityActualValue,
    TorqueDemandValue
};

enum MODE_OF_OPERATION
{
    ProfilePositionmode = 1,
    ProfileVelocitymode = 3,
    ProfileTorquemode = 4,
    Homingmode = 6,
    InterpolatedPositionmode = 7,
    CyclicSynchronousPositionmode = 8,
    CyclicSynchronousVelocitymode = 9,
    CyclicSynchronousTorquemode = 10,
    CyclicSynchronousTorquewithCommutationAngle = 11
};

struct ElmoGoldDevice
{

    struct elmo_gold_rx
    {
        int32_t		targetPosition;
        int32_t		targetVelocity;
        int16_t		targetTorque;
        uint16_t	maxTorque;
        uint16_t	controlWord;
        int8_t      modeOfOperation;
    };

    struct elmo_gold_tx
    {
        int32_t		positionActualValue;
        int32_t		positionFollowingErrrorValue;
        int16_t		torqueActualValue;
        uint16_t	statusWord;
        int8_t		modeOfOperationDisplay;
        int32_t     velocityActualValue;
        int16_t     torqueDemandValue;
    };

    uint16_t alias;
    uint16_t position;
    uint32_t vender_id;
    uint32_t product_code;

    ec_slave_config_t *ecSlaveConfig;

    elmo_gold_rx 	rxPDO;
    elmo_gold_tx 	txPDO;

    uint32_t offsets[ELMO_PDO_ENTRY_SIZE];
};



class EthercatElmoBridge
{

    enum {
        CW_SHUTDOWN = 6,
        CW_SWITCHON = 7,
        CW_ENABLEOP = 15,
        CW_DISABLEOP = 7,
    };


public: // Methods
    EthercatElmoBridge();
    EthercatElmoBridge(int deviceCount);

    virtual ~EthercatElmoBridge();

    int init();
    void deinit();

    void read();
    void write();

    void enableOperation();

    void checkDomainStatus();
    bool controlWordGeneration(uint16_t statusWord, uint16_t &controlWord);

    bool servoOn();
    bool isEnable() {return isEnabled_;}

public: // Variables
    vector<ElmoGoldDevice> device_;
    vector<int32_t> initPosition_;
    vector<bool> isInitPosition_;

private:
    int initMaster();
    int initSlaves();
    int initDomains();
    int configDCMode();
    int activateMaster();

    void registerDomain(ElmoGoldDevice &dev,
                        uint16_t index, uint8_t subindex, uint32_t *offset);
    bool checkAllEnabled() {
        for(int i=0; i<deviceCount_; i++)
        {
            if(isEnabledEach_[i] == false)
            {
                return false;
            }
        }
        return true;
    }

private:
    // EtherCAT
    ec_master_t *ecMaster_;
    ec_master_state_t ecMasterState_;

    ec_domain_t *ecDomain_;
    ec_domain_state_t ecDomainState_;

    ec_pdo_entry_reg_t *ecDomainRegs_;

    int deviceCount_;
    int domainIndex_;
    uint8_t *domainProcessData_;

    bool isEnabled_;
    vector<bool> isEnabledEach_;

};

}

#endif
