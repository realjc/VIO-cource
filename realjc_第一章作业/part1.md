


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

#### 题2：
$$\frac{d\ln(R_1R_2^{-1})^{\vee}}{dR_2}$$
同样采用右扰动模型进行求导。
扰动$\Delta R$对应的李代数为$\varphi$, 对$\varphi$求导有：
$$\frac{d\ln(R_1R_2^{-1})^{\vee}}{dR_2} = \lim_{\varphi \rightarrow 0} \frac{\ln(R_1(R_2\exp(\varphi^{\wedge}))^{-1})^{\vee}-\ln(R_1R_2^{-1})^{\vee}}{\varphi} $$

$$= \lim_{\varphi \rightarrow 0} \frac{\ln(R_1\exp(\varphi^{\wedge})^{-1}R_2^{-1})^{\vee}-\ln(R_1R_2^{-1})^{\vee}}{\varphi} $$

$$= \lim_{\varphi \rightarrow 0} \frac{\ln(R_1R_2^{-1}R_2\exp(\varphi^{\wedge})^{-1}R_2^{-1})^{\vee}-\ln(R_1R_2^{-1})^{\vee}}{\varphi} $$

由$SO_3$的伴随性质$R\exp(p^{\wedge})R^T=\exp((Rp)^{\wedge})$有：
$$\lim_{\varphi \rightarrow 0} \frac{\ln(R_1R_2^{-1}\exp(-R_2\varphi^{\wedge}))^{\vee}-\ln(R_1R_2^{-1})^{\vee}}{\varphi} $$

根据BCH的线性近似：
$$\ln(\exp(\varphi_1^{\wedge})\exp(\varphi_2^{\wedge})) \approx 
\begin{cases}
J_l(\varphi_2)^{-1}\varphi_1+\varphi_2& \varphi_1 \to 0  \\
J_r(\varphi_1)^{-1}\varphi_2+\varphi_1& \varphi_2 \to 0
\end{cases}
$$
有：
$$\lim_{\varphi \rightarrow 0} \frac{\ln(R_1R_2^{-1})^{\vee}-J_r^{-1}(\ln(R_1R_2^{-1})^{\vee})(R_2\varphi)-\ln(R_1R_2^{-1})^{\vee}}{\varphi} =\\
-J_r^{-1}(\ln(R_1R_2^{-1})^{\vee})R_2
$$

