7.6:AHRS��ֲ�ɹ�������yaw���ǲ��ԡ�һ��ʼ��ʼyaw�ܴ󣬵���ת���仯���ԡ����˴��10s��yaw��ʼ��������

7.7��ԭ���Ǵ����ƵĽ��init_mx,init_my,init_mz���󣡲ݣ���Ȼ�ܳɴ����컹���ڵ�һ��kp,ki

7.8���Ǵ����Ƶ����⣬ҪУ׼�����ƣ����������MXgain,MYgain,MZgain
     roll��pitch���仯ʱ���ǻ�Ӱ�쵽yaw���������Ǻܴ󣬿�����10�㷶Χ��

7.9: ����������� ����ֲ������û������·������
     �����ܶ���ң�����������ź��ˣ���ΪIIC�õ���P0�ڣ��Ҳ�������Ĵ���PM0������
     ����Timer_Channel5������˿�P0.5������Ϊ����ˣ�
7.10����λ�����Զ԰��ӽ��и����趨�����ǻ�û���������λ��

7.12: ������У׼ϵ���븱����ͬ
      �����߼�����ͨ������ʼ��PID
      ·�������ȶ�������������WIFI��ǰ20S������߰�����Ϣ���º��������Ϣ������,δ������Bug
7.13��
      ��rollʱ���ֻش��ĵ������д���󣬾���IMU��֤���ų�����ĵ�Ÿ��š�
      ȷ��Ϊ�𶯸��ţ�5hz�˲�Ч��������
7.14:
     ��Сahrs�е�kp,2.5��2.0�������Լ�С
     mpu�˲�ϵ����Ϊ42hzʱЧ���ȽϺ�
     ò���жϴ�ӡ���ݻ����̬�ں����Ӱ�죬����ԭ�����������̬��ʱû�������������
     ��ò��halfT��Ӧ�ö�Ϊ����0.001
     �ڶ�����ԣ�ʧ�ܡ�����������
7.15��
     DMP�Ѿ�ok�ˣ�yaw��ԭ��һ����Ư��10�����Ҳ�����Ӱ��
     ���ϣ�roll: P--4 P--0.13 I--0.006 D--0.012 IMAX--100
     ���ϣ�pitch:P--2 P--0.2  I--0.006 D--0.004 IMAX--50��-7�Ⱦ���ı�IMAXûʲô�仯
     roll\pitch���Ч���ܺã�#define   STABILIZE_ROLL_P       2.0

                     #define   STABILIZE_PITCH_P      2.0

                     #define   STABILIZE_YAW_P        2.0
 
                     #define   RATE_ROLL_P            0.27
                     #define   RATE_ROLL_I            0.006
                     #define   RATE_ROLL_D            0.004
                     #define   RATE_ROLL_IMAX         100.0

                     #define   RATE_PITCH_P           0.2
                     #define   RATE_PITCH_I           0.006
                     #define   RATE_PITCH_D           0.004
                     #define   RATE_PITCH_IMAX        100.0

7.16:
     Z��������£�#define   RATE_YAW_P             0.35
                  #define   RATE_YAW_I             0.006
                  #define   RATE_YAW_D             0.012
                  #define   RATE_YAW_IMAX          50.0
    ��λ������ƫ���ǣ���Ӧ�е���,�Է�ʱ��ƫ45������
    roll\pitch���������Գ鴤�У���Ϊ#define   RATE_ROLL_P            0.2
                                    #define   RATE_ROLL_D            0.001     

                                    #define   RATE_PITCH_P           0.2
                                    #define   RATE_PITCH_D           0.001
    ���ϣ�
          #define   RATE_ROLL_P            0.20
          #define   RATE_ROLL_I            0.006
          #define   RATE_ROLL_D            0.001
          #define   RATE_ROLL_IMAX         100.0

          #define   RATE_PITCH_P           0.20
          #define   RATE_PITCH_I           0.006
          #define   RATE_PITCH_D           0.001
          #define   RATE_PITCH_IMAX        100.0

          #define   RATE_YAW_P             1.0
          #define   RATE_YAW_I             0//.006
          #define   RATE_YAW_D             0//.002
          #define   RATE_YAW_IMAX          50.0
     �ɹ��Էɣ�����pitch���о��һֱ������������ɡ��Ұ�target_pitch = 350;�����򸺷����
               X������Ư
7.18:
     �޸�Y���⻷���������������Ը��ƣ�#define   STABILIZE_ROLL_P       2.0
                                      #define   STABILIZE_ROLL_I       0.5
                                      #define   STABILIZE_ROLL_IMAX    500.0

                                      #define   STABILIZE_PITCH_P      2.0
                                      #define   STABILIZE_PITCH_I      0.5
                                      #define   STABILIZE_PITCH_IMAX   1000.0
     X��ģ������
     �߶Ȳ���ok����ֲ�߶�PIDok
     �޸Ľ��ٶ��ڻ�get_i��������Ч�����������ϵͳ�ȶ�������
7.19��
     �߶ȿ��Ʒ������⻷�����õ��㡣
7.21��
     �߶ȿ������ڸ�Ϊ20ms����������Ϊ1450.
     ��λ���趨�߶�OK
7.22��
     ���µ���yaw��ԭ������̫���ˣ��ᵼ�»�����Ҷ��������ڸ�yaw_pid_out������100��
     �߶��޸�PID�������˵�仯��
     �߶ȿ���Ч�������ԣ��о�D����̫���ˣ����һ������Ÿ������йء������p\d�����������ӡ����������
     d�Ŀ�����һ����ͨ�˲��Ĳ��������Գ��ԡ�
7.23��
     ��������������д��Ѹ�����
     get_d()����һ����ͨ�˲�������һֱû�ӽ�ȥ��Ҳ����˵dԭ��һֱ��������.�޸Ĺ�֮�����Ը��Ƹ߶ȿ���
     �������Ŷ���1520�����ص�ѹ�йأ��Ѿ�ͨ����Ƭ��AD��õ�һ���ص�ѹ������ADת��̫��ʱ�䡣
     �߶�P:0.8
         D:0.4
7.24��
     ���˸�������о����õ㡣
     �������Ÿ����ˣ�һֱ���Ϸ�����״̬��1450basis������ɣ�������״̬��1520�������
7.25��
     �߶Ȳ���
             #define   STABILIZE_HIGH_P        1.0
             #define   STABILIZE_HIGH_I        0.006
             #define   STABILIZE_HIGH_D        0.45 
             #define   STABILIZE_HIGH_IMAX     20
   12.4V���ϻ�������410��11.9V---12.4V֮���������430,11.7V���»�������450.
   ����60cm����30cm,����70cm����50cm�����ø߶���Ӧƽ��
7.26��
     �ֲ����˸߶�PID��Ч��һ�㡣��AD����ص�ѹ�ӽ���������
7.28:
     �����������ˣ�����30cmʱ�ſ�ʼ���λ�ƣ�������ͷ�ӽǸ�Ϊ45�㡣���λ������
     ��λ�������޸�λ��PID���ܡ�����λ�ƹ��ܣ���λ��Ҳ��Ӧ���ܡ�
7.29��
     #define   STABILIZE_POS_X_P         0.25
     #define   STABILIZE_POS_X_I         0
     #define   STABILIZE_POS_X_D         0.39��0.37
     #define   STABILIZE_POS_X_IMAX      2

     #define   STABILIZE_POS_Y_P         0.25
     #define   STABILIZE_POS_Y_I         0
     #define   STABILIZE_POS_Y_D         0.39��0.37
     #define   STABILIZE_POS_Y_IMAX      2
     roll��pitch�ڡ�5�㶶����������Ƶ�п���������λ���ڣ�0,0��ԭ�㶶��
     �����Ŀ�����Ϊ��ʼ��+0.5������Ȼ��X�Ḻ����ɡ�
7.30��
     PCB�����»��ܣ����Ƿɲ��ȣ������ǰ��Ӳ���Ӱ����9150�ڲ�������
7.31��
     ��������д������������
     PCB����Ȼ������
8.1��
     �����õ�·�壬������ַ�������ȷ��ΪPCB��Ӱ����9150���ݡ�
     ���ߵĻ���������ֵ���²��˱飺12.6~12.2    ��360
                                   12.2~11.9    ��380
                                   ����11.9     ��400
     ������ʼ��Ҫ����ʱ�������ϵ��ʼ��ʧ��
     
8.3��
    �ڶ�����������û��������������
    ����AD���ߣ�Ч�����á�
    �ڰ׸����ϸ߶ȶ�ȡ�����⣬������y�Ḻ����ɡ�
8.4:
    ���������ˣ�����֮����ˡ����Ǽ���AD���ܺ���ֵ������������ADֵ����������
    ��������Թ�ϵ���ԡ�
    ��y���ڻ�i���㡣�������Ե���Y�Ḻ����ɡ�
8.6��
    330�������¶���  #define   STABILIZE_HIGH_P        1.5
                     #define   STABILIZE_HIGH_I        0.02
                     #define   STABILIZE_HIGH_D        0.65
                     #define   STABILIZE_HIGH_IMAX     20
    ���������ֵ�ס�����5m������ݣ��ⲻӦ�ð���
    С���ܷʵ���΢�ߵ㻹�ǻᶶ��Ҫ�Ѹ��Ӹ�����΢��
8.7��
    ����ʱX��С������ص�ѹ�������⣡
    �����P��С�˵㣬���ǻửСԲȦ��
    �����������#define   STABILIZE_POS_X_P         0.18
                #define   STABILIZE_POS_X_I         0
                #define   STABILIZE_POS_X_D         0.28
                #define   STABILIZE_POS_X_IMAX      1

                #define   STABILIZE_POS_Y_P         0.18
                #define   STABILIZE_POS_Y_I         0
                #define   STABILIZE_POS_Y_D         0.30
                #define   STABILIZE_POS_Y_IMAX      1
    X�����i��ȷʵ��Ч�������ǻ��ǻ�Ư��������
    ��һ����طŵ�ܿ졣
8.8:
    �ĳɰ���ѡ��ģʽ��key2---����   key3---����
8.10��
    �°��Ӷ��㶨��ס���û�8.8����Ͱ��ӡ���ѹ�͵�ʱ�򶨵���ȣ��Ӵ�Ƕ��⻷P������Ч��
8.11��
    �°��Ӷ������20�飬��ʱ���ǻᵴ������Roll��Ŀ�����Ӧ���ã�
    ���ڵ������������о��������ж�������µ�roll��
8.14��
    ѭ��PID���е㶶������滮�Ѿ����á�
8.20��
     �޸��߾��������޸ļ���������Ľ�����ʱ���޸���yaw�ڻ���P��0.2
8.22:
     �⻷����ȫ����Ϊ4.0,���Ǻ��㣬����yaw��90����Ȼ�ᷭ