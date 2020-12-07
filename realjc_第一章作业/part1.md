


### 公式推导

#### 题1：

$$\frac{d(R^{-1}p)}{dR}$$
这里采用右扰动模型进行求导：
扰动$\Delta R$对应的李代数为$\varphi$, 对$\varphi$求导有：

$$\frac{d(R^{-1}p)}{d\varphi} = \lim_{\varphi \rightarrow 0} \frac{(R \exp(\varphi ^{\wedge} ))^{-1}p-R^{-1}p}{\varphi} $$
由$(AB)^{-1}=B^{-1}A^{-1}$有
$$\lim_{\varphi \rightarrow 0} \frac{\exp(\varphi ^{\wedge} )^{-1}R^{-1}p-R^{-1}p}{\varphi}$$ 
对指数项进行展开:
$$\exp(\varphi ^{\wedge} )= \sum_{n=0}^{\infty} \frac{1}{n!} (\varphi^{\wedge})^n$$
仅保留线性项:
$$\lim_{\varphi \rightarrow 0} \frac{(I+\varphi^{\wedge})^{-1}R^{-1}p-R^{-1}p}{\varphi}$$ 
$$=\lim_{\varphi \rightarrow 0} \frac{(I-\varphi^{\wedge})R^{-1}p-R^{-1}p}{\varphi}$$ 
$$=\lim_{\varphi \rightarrow 0} \frac{-\varphi^{\wedge}(R^{-1}p)}{\varphi}$$ 

由外积公式$a\times b=-b \times a=a^{\wedge}b=-b^{\wedge}a$有：
$$\lim_{\varphi \rightarrow 0} \frac{(R^{-1}p)^{\wedge}\varphi}{\varphi}\\
=(R^{-1}p)^{\wedge}$$ 

