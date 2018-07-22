#include "hardware/ethercat_elmo.h"
#include <native/timer.h>
#include <string>

using namespace std;

namespace EtherCAT_Elmo
{


ec_pdo_entry_info_t elmo_gold_pdo_entries[] =
{
    // 0x1605
    {0x607A,	0,	32},	// Target Position          0
    {0x60FF,	0,	32},	// Target Velocity
    {0x6071,	0,	16},	// Target Torque
    {0x6072,	0,	16},	// Max Torque
    {0x6040,	0,	16},	// Control Word
    {0x6060,	0,	8},     // Mode Of Operation        5
    {0,    0,  8},

    // 0x1A04
    {0x6064,	0,	32},	// Position Actual Value      6
    {0x60F4,	0,	32},	// Position Following Errror Actual Value
    {0x6077,	0,	16},	// Torque Actual Value
    {0x6041,	0,	16},	// Status Word
    {0x6061,	0,	8},     // Mode Of Operation Display   10
    {0,    0,  8},

    // 0x1A11
    {0x606C,    0,  32},    // Velocity Actual Value
    // 0x1A12
    {0x6074,    0,  16},    // Torque Demand value

};

ec_pdo_info_t elmo_gold_pdos[] =
{
    // output
    {0x1605,	7,	elmo_gold_pdo_entries + 0},

    // input
    {0x1A04,	6,	elmo_gold_pdo_entries + 7},
    {0x1A11,    1,  elmo_gold_pdo_entries + 13},
    {0x1A12,    1,  elmo_gold_pdo_entries + 14},
};

ec_sync_info_t elmo_gold_syncs[] =
{
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT	, 0, NULL},
    {2, EC_DIR_OUTPUT, 1, elmo_gold_pdos + 0},
    {3, EC_DIR_INPUT , 3, elmo_gold_pdos + 1},
    {0xff}
};



/************************************************/
/* Class Implementation                         */
/************************************************/

EthercatElmoBridge::EthercatElmoBridge() :
    ecMaster_(NULL), ecDomain_(NULL), deviceCount_(0), ecDomainRegs_(NULL), domainIndex_(0), isEnabled_(false)
{}

EthercatElmoBridge::EthercatElmoBridge(int deviceCount) :
    ecMaster_(NULL), ecDomain_(NULL), deviceCount_(deviceCount), domainIndex_(0), isEnabled_(false)
{
    ecDomainRegs_ = new ec_pdo_entry_reg_t[ELMO_PDO_ENTRY_SIZE * deviceCount + 10];

    device_.resize(deviceCount);
    isEnabledEach_.resize(deviceCount, false);
    initPosition_.resize(deviceCount);
    isInitPosition_.resize(deviceCount, false);
    for(int i=0; i<deviceCount; i++)
    {
        device_[i].alias = 0;
        device_[i].position = i;
        device_[i].vender_id = ELMO_GOLD_VENDER_ID;
        device_[i].product_code = ELMO_GOLD_PRODUCT_CODE;
    }
}

EthercatElmoBridge::~EthercatElmoBridge()
{
    deinit();

    if(ecDomainRegs_)
        delete ecDomainRegs_;
}


int EthercatElmoBridge::init()
{
    if(deviceCount_ == 0)
    {
        cout << "Elmo EtherCAT Init Failed: Please set the number of device." << endl;
        return -1;
    }

    initMaster();
    initSlaves();
    initDomains();
    //configDCMode();
    activateMaster();

}

void EthercatElmoBridge::deinit()
{
    ecrt_master_deactivate(ecMaster_);
}


void EthercatElmoBridge::read()
{
    ecrt_master_receive(ecMaster_);
    ecrt_domain_process(ecDomain_);

    for(int i=0; i<deviceCount_; i++)
    {
        ElmoGoldDevice &dev = device_[i];
        dev.rxPDO.targetPosition = EC_READ_S32(domainProcessData_ + dev.offsets[TargetPosition]);
        dev.rxPDO.targetVelocity = EC_READ_S32(domainProcessData_ + dev.offsets[TargetVelocity]);
        dev.rxPDO.targetTorque = EC_READ_S16(domainProcessData_ + dev.offsets[TargetTorque]);
        dev.rxPDO.maxTorque = EC_READ_U16(domainProcessData_ + dev.offsets[MaxTorque]);
        dev.rxPDO.controlWord = EC_READ_U16(domainProcessData_ + dev.offsets[ControlWord]);
        dev.rxPDO.modeOfOperation = EC_READ_S8(domainProcessData_ + dev.offsets[ModeOfOperation]);

        dev.txPDO.positionActualValue =             EC_READ_S32(domainProcessData_ + dev.offsets[PositionActualValue]);
        dev.txPDO.positionFollowingErrrorValue =    EC_READ_S32(domainProcessData_ + dev.offsets[PositionFollowingErrrorValue]);
        dev.txPDO.torqueActualValue =               EC_READ_S16(domainProcessData_ + dev.offsets[TorqueActualValue]);
        dev.txPDO.statusWord =                      EC_READ_U16(domainProcessData_ + dev.offsets[StatusWord]);
        dev.txPDO.modeOfOperationDisplay =          EC_READ_S8 (domainProcessData_ + dev.offsets[ModeOfOperation]);
        dev.txPDO.velocityActualValue =             EC_READ_S32(domainProcessData_ + dev.offsets[VelocityActualValue]);
        dev.txPDO.torqueDemandValue =               EC_READ_S16(domainProcessData_ + dev.offsets[TorqueDemandValue]);
        // dev.txPDO.analogInput =                     EC_READ_S16(domainProcessData_ + dev.offsets[AnalogInput]);
    }

    RTIME _masterTime = rt_timer_read();

    ecrt_master_application_time(ecMaster_, _masterTime);
    ecrt_master_sync_reference_clock(ecMaster_);
    ecrt_master_sync_slave_clocks(ecMaster_);

    //    ecrt_domain_queue(ecDomain_);
    //    ecrt_master_send(ecMaster_);
}
void EthercatElmoBridge::write()
{
    //ecrt_master_receive(ecMaster_);
    //ecrt_domain_process(ecDomain_);

    for(int i=0; i<deviceCount_; i++)
    {
        ElmoGoldDevice &dev = device_[i];

        //isEnabledEach_[i] = controlWordGeneration(dev.txPDO.statusWord,dev.rxPDO.controlWord);
        //isEnabled_= checkAllEnabled();

        /*
    rt_printf("ctrlwrd = %d stats = %d ",dev.rxPDO.controlWord, dev.txPDO.statusWord);
    if(isEnabled_)
        rt_printf("%d -> Enabled\n",i);
    else
        rt_printf("%d -> Disabled\n",i);
*/
        (void)controlWordGeneration(dev.txPDO.statusWord,dev.rxPDO.controlWord);
        EC_WRITE_S32(domainProcessData_ + dev.offsets[TargetPosition], dev.rxPDO.targetPosition);
        EC_WRITE_S32(domainProcessData_ + dev.offsets[TargetVelocity], dev.rxPDO.targetVelocity);
        EC_WRITE_S16(domainProcessData_ + dev.offsets[TargetTorque], dev.rxPDO.targetTorque);
        EC_WRITE_U16(domainProcessData_ + dev.offsets[MaxTorque], dev.rxPDO.maxTorque);
        EC_WRITE_U16(domainProcessData_ + dev.offsets[ControlWord], dev.rxPDO.controlWord);
        EC_WRITE_S8(domainProcessData_ + dev.offsets[ModeOfOperation], dev.rxPDO.modeOfOperation);
        //rt_printf("%d offset=%d\n",i,dev.offsets[ModeOfOperation]);
    }

    ecrt_domain_queue(ecDomain_);
    ecrt_master_send(ecMaster_);
}

void EthercatElmoBridge::enableOperation()
{/*
    ros::Rate r(100);
    while(ros::ok())
    {
        bool isDone = true;
        read();
        for(int i=0; i<deviceCount_; i++)
        {
            if( !controlWordGeneration(device_[i].txPDO.statusWord,
                                      device_[i].rxPDO.controlWord) )
            {
                isDone = false;
            }
        }
        write();
        if(isDone == true) return ;
        r.sleep();
    }*/
}


void EthercatElmoBridge::checkDomainStatus()
{
    ec_domain_state_t ds;

    ecrt_domain_state(ecDomain_, &ds);

    printf("Domain1: WC %u.\n", ds.working_counter);
    printf("Domain1: State %u.\n", ds.wc_state);
}

int EthercatElmoBridge::initMaster()
{
    ecMaster_ = ecrt_request_master(0);
    if (!ecMaster_){
        cout << "No Master" << endl;
        return -1;
    }
    return 0;
}

int EthercatElmoBridge::initSlaves()
{
    for(int i=0; i<deviceCount_; i++)
    {
        ElmoGoldDevice &dev = device_[i];

        if (!(dev.ecSlaveConfig = ecrt_master_slave_config(
                  ecMaster_, dev.alias,dev.position,
                  dev.vender_id, dev.product_code))) {
            cout << "Failed to get slave configuration." << endl;
            return -1;
        }

        ecrt_slave_config_pdo_assign_clear(dev.ecSlaveConfig, 2);
        ecrt_slave_config_pdo_assign_clear(dev.ecSlaveConfig, 3);

        cout << "Configuring PDOs..." << endl;
        if (ecrt_slave_config_pdos(dev.ecSlaveConfig, EC_END, elmo_gold_syncs)) {
            cout << "Failed to configure PDOs." << endl;
            return -1;
        }
    }
    return 0;
}

int EthercatElmoBridge::initDomains()
{
    // domain creation
    ecDomain_ = ecrt_master_create_domain(ecMaster_);
    if (!ecDomain_){
        cout << "No Domain" << endl;
        return -1;
    }

    // domain list registration
    for(int i=0; i<deviceCount_; i++)
    {
        ElmoGoldDevice &dev = device_[i];
        for(int j=0; j<ELMO_PDO_ENTRY_SIZE-2; j++)
        {

            // registerDomain(dev,elmo_gold_pdo_entries[j].index,
            //              elmo_gold_pdo_entries[j].subindex, &dev.offsets[j]);

            if(j>=11)
            {
                registerDomain(dev,elmo_gold_pdo_entries[j+2].index,
                        elmo_gold_pdo_entries[j+2].subindex, &dev.offsets[j]);

            } else if (j>=6)
            {
                registerDomain(dev,elmo_gold_pdo_entries[j+1].index,
                        elmo_gold_pdo_entries[j+1].subindex, &dev.offsets[j]);
            }
            else
            {
                registerDomain(dev,elmo_gold_pdo_entries[j].index,
                               elmo_gold_pdo_entries[j].subindex, &dev.offsets[j]);
            }

        }
    }

    // domain registration
    if (ecrt_domain_reg_pdo_entry_list(ecDomain_, ecDomainRegs_))
    {
        cout << "PDO entry registration failed!" << endl;
        return -1;
    }

    return 0;
}

int EthercatElmoBridge::configDCMode()
{

    for(int i=0; i<deviceCount_; i++)
    {
        // configure SYNC signals for this slave
        ecrt_slave_config_dc(device_[i].ecSlaveConfig, 0x0300, PERIOD_NS, 0, 0, 0);

    }


    int ret = ecrt_master_select_reference_clock(ecMaster_, device_[0].ecSlaveConfig);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to select reference clock: \n");
        return ret;
    }


}

int EthercatElmoBridge::activateMaster()
{
    if (ecrt_master_activate(ecMaster_)){
        return -1;
        cout << "Activating master failed!" << endl;
    }

    //usleep(1000);

    // domain process data
    if (!(domainProcessData_ = ecrt_domain_data(ecDomain_)))
    {
        cout << "Cannot get a domain data" << endl;
        return -1;
    }

    return 0;
}





void EthercatElmoBridge::registerDomain(ElmoGoldDevice &dev,
                                        uint16_t index, uint8_t subindex, uint32_t *offset)
{
    ecDomainRegs_[domainIndex_].alias = dev.alias;
    ecDomainRegs_[domainIndex_].position = dev.position;
    ecDomainRegs_[domainIndex_].vendor_id = dev.vender_id;
    ecDomainRegs_[domainIndex_].product_code = dev.product_code;
    ecDomainRegs_[domainIndex_].index = index;
    ecDomainRegs_[domainIndex_].subindex = subindex;
    ecDomainRegs_[domainIndex_].offset = offset;
    ecDomainRegs_[domainIndex_].bit_position = NULL;

    // rt_printf("%d domain %d %d %d %d %d %d %d\n", domainIndex_, dev.alias, dev.position, dev.vender_id, dev.product_code, index, subindex, offset);
    domainIndex_++;
}

bool EthercatElmoBridge::servoOn()
{
    bool tempready[deviceCount_];
    for (int i=0; i<deviceCount_; i++)
    {
        tempready[i]=false;
    }

    read();

    /*ecrt_master_receive(ecMaster_);
    ecrt_domain_process(ecDomain_);

    // TODO: Read servos' status to Axes' Status Word

    for (int i=0; i<NUM_MOTION_AXIS; ++i)
    {
        dev.txPDO.positionActualValue =             EC_READ_S32(domainProcessData_ + dev.offsets[PositionActualValue]);
        dev.txPDO.statusWord =                      EC_READ_U16(domainProcessData_ + dev.offsets[StatusWord]);

        _master.readDCDomain<INT32>(&(Axis[i].OutParam.PositionActualValue), 0, i, 0x6064, 0x00);
        _master.readDCDomain<UINT16>(&(Axis[i].OutParam.StatusWord), 0, i, 0x6041, 0x00);
        //rt_printf("Servo %d Status Word 0x%x \n", i, Axis[i].OutParam.StatusWord);

        data[i] = Axis[i].InParam.TargetPosition = (float) Axis[i].OutParam.PositionActualValue;
    }
*/
    for(int i=0; i<deviceCount_; i++)
    {
        ElmoGoldDevice &dev = device_[i];

        //init_pos[i] = dev.txPDO.positionActualValue;

        tempready[i] = controlWordGeneration(dev.txPDO.statusWord,dev.rxPDO.controlWord);
        if(tempready[i] == true)
        {
            if(isInitPosition_[i])
            {
                dev.rxPDO.targetPosition = initPosition_[i];
            }
            else
            {
                initPosition_[i] = dev.txPDO.positionActualValue;
                isInitPosition_[i] = true;
            }
        }
        else
        {
            dev.rxPDO.targetPosition = dev.txPDO.positionActualValue;
        }
    }

    write();

    for (int i=0; i<deviceCount_; i++)
    {
        if(!tempready[i])
        {
            isEnabled_ = false;
            return false;
        }
    }
    isEnabled_ = true;
    return true;

    // TODO: Compute and Write Control Word + Mode of Operation to servos
    // Note that you can call _computeCtrlWrd(StatusWord) in this case but you have to take care of the tempready[i] yourself
    // tempready[i] indicates that the ith servo is ready for operation

    /*
    for (int i=0; i<deviceCount_; i++){
        if (!tempready[i]){

            switch (Axis[i].OutParam.StatusWord & 0x6F)
            {
            case 0:	// No Initialization -> Request Initialization
                Axis[i].InParam.ControlWord = (Axis[i].InParam.ControlWord & 0xFF70) | 0;
                break;

            case 0x40:
            case 0x60: // Initialization Completion -> Request Main Circuit Power OFF
                Axis[i].InParam.ControlWord = (Axis[i].InParam.ControlWord & 0xFF70) | 0x06;
                break;

            case 0x21: // Main Circuit Power OFF -> Request Servo Ready
                Axis[i].InParam.ControlWord = (Axis[i].InParam.ControlWord & 0xFF70) | 0x07;
                tempready[i]=1;
                break;

            case 0x23: //Servo Ready -> Request Servo ON
                Axis[i].InParam.ControlWord = (Axis[i].InParam.ControlWord & 0xFF70) | 0x0F;
                tempready[i]=1;
                break;

            case 0x27: // Servo ON State (Ready for operation)
                tempready[i]=1;
                break;
            case 0x2F: // In Abnormal Process
                break;

            case 0x28: // Abnormal State -> Request Initialization
                Axis[i].InParam.ControlWord = (Axis[i].InParam.ControlWord & 0xFF70) | 0x80;
                break;
            }

            //rt_printf("Servo %d Status 0x%x Control 0x%x\n", i, Axis[i].OutParam.StatusWord );

            _master.writeDCDomain<INT32>(Axis[i].InParam.TargetPosition, 0, i, 0x607A, 0x00);
            _master.writeDCDomain<INT32>(Axis[i].InParam.TargetTorque, 0, i, 0x607A, 0x00);
            _master.writeDCDomain<UINT16>(Axis[i].InParam.ControlWord, 0, i, 0x6040, 0x00);
            _master.writeDCDomain<UINT8>(_modeOp, 0, i, 0x6060, 0x00);
        }
    }


    endCommDC();


    for (int i=0; i<NUM_MOTION_AXIS; ++i)
    {
        if (!tempready[i])
        {
            _systemReady = false;
            return -1;
        }
    }
    _systemReady = true;


    return 0;
    */
}


bool EthercatElmoBridge::controlWordGeneration(uint16_t statusWord, uint16_t &controlWord)
{
    //uint16_t bit03 = statusWord & 0xF;
    //uint16_t bit5 = statusWord & 32;    // Quick Stop
    //uint16_t bit6 = statusWord & 64;    // Switch on disabled

    const int FAULT_BIT = 3;
    const int OPERATION_ENABLE_BIT = 2;
    const int SWITCHED_ON_BIT = 1;
    const int READY_TO_SWITCH_ON_BIT = 0;
    //cout << "status = " << statusWord << endl;
    if(!(statusWord & (1 << OPERATION_ENABLE_BIT)))
    {
        if(!(statusWord & (1 << SWITCHED_ON_BIT)))
        {
            if(!(statusWord & (1 << READY_TO_SWITCH_ON_BIT)))
            {
                if(statusWord & (1 << FAULT_BIT))
                {
                    //rt_printf("Fault Reset\n");
                    controlWord = 0x80;
                    return false;
                }
                else
                {
                    //rt_printf("Transition: Switch on disabled -> Ready to switch on\n");
                    controlWord = CW_SHUTDOWN;
                    return false;
                }
            }
            else
            {
                //rt_printf("Transition: Ready to switch on -> Switch on\n");
                controlWord = CW_SWITCHON;
                return true;
            }
        }
        else
        {
            //rt_printf("Operation enabled\n");
            controlWord = CW_ENABLEOP;
            return true;
        }
    }
    else
    {
        controlWord = CW_ENABLEOP;
        return true;
    }
    controlWord = 0;
    return false;

    /*
    switch (bit03)
    {
    case 0:
        if (bit6 == 0) // Not ready to switch on
        {
            //cout << "Not ready to switch on" << endl;
            return false;
        }
        else    // Switch on disabled
        {
            controlWord = CW_SHUTDOWN;
            //cout << "Transition: Switch on disabled -> Ready to switch on" << endl;
        }
        break;
    case 1:     // Ready to switch on
        controlWord = CW_SWITCHON;
        //cout << "Transition: Ready to switch on -> Switch on" << endl;
        break;
    case 3:     // Switch on
        controlWord = CW_ENABLEOP;
        //cout << "Transition: Switch on -> Operation enabled"<<endl;
        return true;
        break;
    case 7:
        if(bit5) // Operation enabled
        { return true; }
        else          // Quick stop active
        { controlWord = CW_ENABLEOP;
        //cout << "QUICK STOP?" << endl;
        }
        break;
    case 8:
        controlWord = 0x80;
        //cout << "Transition: 1111 -> fault reset" << endl;
        break;
    default:
        controlWord = 0x80;
        cout << "Fault reset" << endl;
        break;
    }
    return false;
    */
}


}
