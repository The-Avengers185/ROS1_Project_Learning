#ifndef DBX_PLUGINS_H_
#define DBX_PLUGINS_H_
#include "demo03_plugin/dbx_base.h"

namespace dbx_plugins_ns {
    //三边形
    class SanBian: public dbx_base_ns::Dbx_Base{
        private:
            double side_length;
        public:
            SanBian(){
                side_length = 0.0;
            }
            void init(double side_length){
                this->side_length = side_length;
            }
            double getLength(){
                return side_length * 3;
            }
    };
    
    //四边形
    class SiBian: public dbx_base_ns::Dbx_Base{
        private:
            double side_length;
        public:
            SiBian(){
                side_length = 0.0;
            }
            void init(double side_length){
                this->side_length = side_length;
            }
            double getLength(){
                return side_length * 4;
            }
    };
};

#endif
