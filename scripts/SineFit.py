import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize


def test(a,b,noise_gain=0):
    n = 100
    x = np.random.rand(n) * 6.28
    noise = noise_gain * (np.random.rand(n) - .5)
    y = a* np.cos(2*x) + b*np.sin(2*x) + noise
    
    sf = SineFit(x,y)
    sf.fit()
    
    m = (a**2+b**2)**(.5)
    print 'fractional err a: %f, b: %f'%( (sf.cos-a)/m, (sf.sin-b)/m )
    
    x = np.linspace(-3,3,600)
    y_predicted = sf.predict(x)
    sf.plot_data()
    sf.plot_fit()
    plt.plot(x,y_predicted,'--')
    
    return sf

class SineFitFactory():
    def __init__(self, exp_name=None):
        self.exp_name = exp_name

    def create_sine_fit(self, datax, datay, fname=None):
        return SineFit(datax, datay, fname, self.exp_name)
    
class SineFit():
    def __init__(self,datax,datay, fname=None, exp_name=None):
        self.datax = datax
        self.datay = datay
        
        self.cos = 0
        self.sin = 0
        self.freq = 0
        self.bias = 0
        self.optimizer_ret = None

        if fname is not None:
            if exp_name is not None:
                fname = exp_name + '.' + fname

            f = open(fname + '.datax', 'w')
            np.save(f, self.datax)
            f.close()
            f = open(fname + '.datay', 'w')
            np.save(f, self.datay)
            f.close()

    def objective(self,params):
        #technically not an objective function.
        (a,b,w,c) = params
        prediction = a * np.cos ( w * self.datax ) + b * np.sin(w * self.datax) + c
        error = prediction - self.datay 
        return error

    def fit(self,fix_freq=None,fix_bias=None):
        if fix_freq is None and fix_bias is None:
            r = scipy.optimize.leastsq(self.objective,[0,0,1,0])  
            (a,b,w,c) = r[0]
            #ensure frequency is positive (put parameters in a canonical form)
            if w<0:
                w = -w
                b = -b
            (self.cos,self.sin,self.freq,self.bias) = (a,b,w,c)
        elif fix_freq is not None and fix_bias is None:
            def obj(p):
                a,b,c = p
                return self.objective([a,b,fix_freq,c])
                
            r = scipy.optimize.leastsq(obj,[0,0,0])  
            (a,b,c) = r[0]
            self.optimizer_ret = r
            (self.cos,self.sin,self.freq,self.bias) = (a,b,fix_freq,c)
        elif fix_freq is None and fix_bias is not None:
            def obj(p):
                a,b,w = p
                return self.objective([a,b,w,fix_bias])
                
            r = scipy.optimize.leastsq(obj,[0,0,0])  
            (a,b,w) = r[0]            
            (self.cos,self.sin,self.freq,self.bias) = (a,b,w,fix_bias)
        else:
            def obj(p):
                a,b = p
                return self.objective([a,b,fix_freq,fix_bias])
                
            r = scipy.optimize.leastsq(obj,[0,0])  
            (a,b) = r[0]            
            (self.cos,self.sin,self.freq,self.bias) = (a,b,fix_freq,fix_bias)            

    def predict(self,x):
        y = self.cos * np.cos (self.freq * x) + self.sin * np.sin(self.freq * x) + self.bias
        return y
            
    def plot_fit(self,color='k'):
        mi = np.min(self.datax)
        ma = np.max(self.datax)
        x = np.arange(mi,ma,.01)

        y=self.predict(x)
        plt.plot(x,y,'-',color=color,lw=2, zorder=2)
        
        plt.axhline(self.bias,color=color)      
        plt.axvline(self.zerocross(),color=color)

    def zerocross(self):
        phi = np.arctan2(self.cos,self.sin)  #at t = -phi, the argument of the sinusoid is zero
        zcross = -phi/self.freq
        return zcross        

    def amplitude(self):
        return (self.cos**2+self.sin**2)**(.5)
        
    def plot_data(self,color='m'):
        plt.plot( self.datax , self.datay , '.',color=color,alpha=.5, zorder=0)
 
    def plot_residual(self):
        residual = self.datay - self.predict(self.datax)
        plt.plot(self.datax,residuals,'.')
