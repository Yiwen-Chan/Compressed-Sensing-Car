
class IncrementalPID(PID):
    '''
    增量式PID
    '''
    def __init__(self, kp, ki=0, kd=0, target=0, max_result=None, min_result=None):
        # 调用父类的构造器
        super().__init__(kp=kp, ki=ki, kd=kd, target=target)
        # 控制量
        self.old_bias = 0  # 前一次的误差 err_k-2
        self.max_result = max_result
        self.min_result = min_result

    def reset(self):
        self.old_bias = 0
        self.pre_bias = 0
        self.cur_bias = 0
        self._target = 0
    def target(self, value=None):
        '''获取或设置PID'''
        if value is None:
            return self._target
        self._target = value
    def update(self, value):
        '''
        增量式PID更新公式
            Result +=Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
        增量式PID只与当前的误差还有上两次的误差有关
        e(k)：cur_bias
        e(k-1)：pre_bias
        e(k-2)：old_bias
        '''

        # 计算当前的err
        self.cur_bias = value - self._target

        # 更新控制量的值
        self.result += self.kp*(self.cur_bias - self.pre_bias) + \
            self.ki*self.cur_bias + \
            self.kd*(self.cur_bias -2*self.pre_bias + self.old_bias)

        # 更新误差历史
        self.pre_bias, self.old_bias = self.cur_bias, self.pre_bias

        return self.result
