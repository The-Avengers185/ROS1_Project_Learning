#ifndef DBX_BASE_H_
#define DBX_BASE_H_

namespace dbx_base_ns{

    /*
        注意:必须保证基类中有无参构造

    */
    class Dbx_Base{
        protected:
            Dbx_Base(){};

        public:
            //计算周长的函数
            virtual double getLength() = 0;
            //初始化边长函数
            virtual void init(double side_length) = 0;
        
    };
};

#endif

