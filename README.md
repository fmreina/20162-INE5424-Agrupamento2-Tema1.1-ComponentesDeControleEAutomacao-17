# 20162-INE5424-Agrupamento2-Tema1.1-ComponentesDeControleEAutomacao-17
Trabalho final da matéria de Sistemas Operacionais II.

* ####Relatório para entrega de 01/11 em: [Google Docs] (https://docs.google.com/document/d/16eYw_w1bm3-xb4zV7cB2e5bX3gjFhLIdkE74u10BXgE/edit?usp=sharing)

* Referência [EPOS]: (http://epos.lisha.ufsc.br/HomePage) => EPOS2 em: [teaching/ine5424](https://epos.lisha.ufsc.br/svn/teaching/ine5424/)

* Implementação inicial do controlador [PID] (https://github.com/fmreina/PIDController)

### - Requisitos:
* RF01. Modelar controladores de uma variável como componentes do sistema operacional (pelo menos P, PD, PI, PID)
* RF02. Implementar a abstração de controlador de uma variável num sistema operacional (com realizações de pelo menos P, PD, PI, e PID)
* RF03. Criar testes unitários para cada controlador desenvolvido, respondendo a estímulos clássico, como o degrau unitário
* RF04. Avaliar cada controlador desenvolvido, mensurando parâmetros de reposta dos controladores, como estabilidade, desempenho no transitório e no regime
* RF05. Implementar o controle de uma variável num sistema de malha fechada qualquer (ex: controle de iluminação)
* RNF01. Os componentes devem ser implementados no sistema operacional EPOS 2

#### Modelo inicial
![modelo](https://github.com/fmreina/PIDController/blob/master/images/modelController.png)

#### PID
![PID](https://github.com/fmreina/PIDController/blob/master/images/PID.png)
