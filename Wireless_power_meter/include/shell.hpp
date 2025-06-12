/*
 * @LastEditors: qingmeijiupiao
 * @Description: 串口命令行相关
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-12-10 16:27:18
 */
#ifndef SHELL_HPP
#define SHELL_HPP
#include "static/SimpleSerialShell.hpp"
#include "static/HXCthread.hpp"
#include "static/HXC_NVS.hpp"
#include "static/POWERMETER.hpp"
namespace SHELL{


    // 初始化shell
    void shell_init(void){
        shell.attach(Serial);


        /*↓↓↓↓↓↓↓↓在下面添加命令↓↓↓↓↓↓↓↓↓*/

        // info 打印信息
        shell.addCommand(F("info 显示编译信息"),[](int argc, char** argv){
            shell.println(F( "Built " __DATE__));
            return 0;
        });

        // set_resistance 修改采样电阻配置
        shell.addCommand(F("set_resistance 设置采样电阻值,单位mΩ"),[](int argc, char** argv){
            if(argc!=2){
                shell.println(F("参数错误"));
                return -1;
            };
            float res=atof(argv[1]);
            if(res<=0||res>10){
                shell.println(F("参数错误"));
                return -1;
            }
            //赋值
            POWERMETER::sample_resistance=res;
            return 0;
        });

        // esp_now_secret_key 修改esp_now密钥
        shell.addCommand(F("set_esp_now_secret_key 设置esp_now密钥"),[](int argc, char** argv){
            if(argc!=2){
                shell.println(F("参数错误"));
                return -1;
            };
            uint16_t key=atoi(argv[1]);
            if(key<=0||key>0xFFFF){
                shell.println(F("参数错误"));
                shell.printf("你输入的:0x%X",key);
                return -1;
            }
            shell.println("修改成功");
            shell.printf("新密钥:0x%X",key);
            WIRELESSCTRL::esp_now_secret_key=key;
            change_secret_key(WIRELESSCTRL::esp_now_secret_key);
            return 0;
        });

        // get_esp_now_secret_key 获取esp_now密钥 
        shell.addCommand(F("get_esp_now_secret_key 获取esp_now密钥"),[](int argc, char** argv){
            shell.print("当前密钥:  ");
            uint16_t key=WIRELESSCTRL::esp_now_secret_key.read();
            shell.printf("0x%X\n",key);
            return 0;
        });



    }

    // shell线程
    HXC::thread<void> shell_thread([](){
        shell_init();
        while (1){
            shell.executeIfInput();
            delay(10);
        }
    });


}



#endif