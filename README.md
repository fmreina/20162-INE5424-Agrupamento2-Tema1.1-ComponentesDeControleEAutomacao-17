# 20162-INE5424-Agrupamento2-Tema1.1-ComponentesDeControleEAutomacao-17
Trabalho final da matéria de Sistemas Operacionais II.

* ####Relatório para entrega de 06/12 em: [Google Docs : Refatoração] (https://docs.google.com/document/d/1zX6SwOYkBhrT77jhIFm0wuz53DCrL8yPcxdTGasuGsg/edit?usp=sharing)

* Relatório para entrega de 08/11 em: [Google Docs : Final] (https://docs.google.com/document/d/1ogb7eUfPUyE0YE55DE1nte3eipcnUsg_0L54t7ti5BM/edit?usp=sharing)

* Relatório para entrega de 01/11 em: [Google Docs : Parcial] (https://docs.google.com/document/d/16eYw_w1bm3-xb4zV7cB2e5bX3gjFhLIdkE74u10BXgE/edit?usp=sharing)

* Referência [EPOS](http://epos.lisha.ufsc.br/HomePage) => EPOS2 em: [teaching/ine5424](https://epos.lisha.ufsc.br/svn/teaching/ine5424/)

* Implementação inicial do controlador [PID] (https://github.com/fmreina/PIDController)

### - Requisitos:
* [x] RF01. Modelar controladores de uma variável como componentes do sistema operacional (pelo menos P, PD, PI, PID)
* [x] RF02. Implementar a abstração de controlador de uma variável num sistema operacional (com realizações de pelo menos P, PD, PI, e PID)
* [x] RF03. Criar testes unitários para cada controlador desenvolvido, respondendo a estímulos clássico, como o degrau unitário
* [x] RF04. Avaliar cada controlador desenvolvido, mensurando parâmetros de reposta dos controladores, como estabilidade, desempenho no transitório e no regime
* [ ] RF05. Implementar o controle de uma variável num sistema de malha fechada qualquer (ex: controle de iluminação)
* [x] RNF01. Os componentes devem ser implementados no sistema operacional EPOS 2

#### - Modelo inicial
![modelo](https://github.com/fmreina/PIDController/blob/master/images/modelController.png)

#### - PID
![PID](https://github.com/fmreina/PIDController/blob/master/images/PID.png)

#### - Modelo implementado
decidiu-se implementar como indicado no modelo, no entando os métodos dos controladores seguem a ideia do modelo inicial, de forma que os controladores PI, PD e PID continuam sendo composições dos controladores P, I e D.
![modeloImplementado](https://github.com/fmreina/20162-INE5424-Agrupamento2-Tema1.1-ComponentesDeControleEAutomacao-17/blob/pThread/images/implementedModel.png)

#### - Modelo Refatorado
![modelorefatorado](https://github.com/fmreina/20162-INE5424-Agrupamento2-Tema1.1-ComponentesDeControleEAutomacao-17/blob/master/images/refactoredModel.png)

### - Avaliação dos controladores (resultado dos testes de pid_controller_test.cc)
O desempenho dos controladores foram testados com os seguintes parametros:
* kp = 1.600
* ki = 1.200
* kd = 0.200
* dt = 0.500
* setpoint = 1.000

![ptest](https://github.com/fmreina/20162-INE5424-Agrupamento2-Tema1.1-ComponentesDeControleEAutomacao-17/blob/pThread/images/pController.png)
![itest](https://github.com/fmreina/20162-INE5424-Agrupamento2-Tema1.1-ComponentesDeControleEAutomacao-17/blob/pThread/images/iController.png)
![dtest](https://github.com/fmreina/20162-INE5424-Agrupamento2-Tema1.1-ComponentesDeControleEAutomacao-17/blob/pThread/images/dController.png)
![pitest](https://github.com/fmreina/20162-INE5424-Agrupamento2-Tema1.1-ComponentesDeControleEAutomacao-17/blob/pThread/images/piController.png)
![pdtest](https://github.com/fmreina/20162-INE5424-Agrupamento2-Tema1.1-ComponentesDeControleEAutomacao-17/blob/pThread/images/pdController.png)
![pidtest](https://github.com/fmreina/20162-INE5424-Agrupamento2-Tema1.1-ComponentesDeControleEAutomacao-17/blob/pThread/images/pidController.png)
