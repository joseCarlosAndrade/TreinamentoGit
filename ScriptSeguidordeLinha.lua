function sysCall_init()
    -- CODIGO DE INICIALIZAÇÃO - atribuição dos objetos do cenario as variaveis
    motorEsquerdo=sim.getObjectHandle('Roda_1')
    motorDireito=sim.getObjectHandle('Roda_2')
    sensor0=sim.getObjectHandle('sensor0')
    sensor1=sim.getObjectHandle('sensor1')
    sensor2=sim.getObjectHandle('sensor2')
    sensor3=sim.getObjectHandle('sensor3')
    sensor4=sim.getObjectHandle('sensor4')
    camera=sim.getObjectHandle('camera')
    
	--VARIAVEIS GLOBAIS INICIAIS
    vMotor=20
    contadorVolta=0
    auxiliar=true
    cruzamento=false
    parar=false
    auxTempo=true
    erro=0
    erroanterior=0
    
    integral = 0
    derivativo = 0
    
    kp=2
    ki=0.5
    kd=1
    
    --VELOCIDADE INICIAL DOS MOTORES
    sim.setJointTargetVelocity(motorEsquerdo, vMotor)
    sim.setJointTargetVelocity(motorDireito, vMotor)
    

	--FUNÇAO PARA CURVAS
    function virar(volta)
    	if (volta == 1 and data2[14]>0.5) then
        
         	vMotorl = -10
      	 	vMotorr = 5
       	 	sim.setJointTargetVelocity(motorEsquerdo, vMotorl)
       	 	sim.setJointTargetVelocity(motorDireito, vMotorr)
       
      	elseif (volta == 2 and data2[14]>0.5) then
        	vMotorl = 5
        	vMotorr = -10
        	sim.setJointTargetVelocity(motorEsquerdo, vMotorl)
        	sim.setJointTargetVelocity(motorDireito, vMotorr)
    	end
    end
    
   
	-- GERA A ENTRADA DE ERRO
    function erroCalc()
    
            if (data0[11]>0.5 and data1[11]>0.5 and data2[11]>0.5 and data3[11]>0.5 and data4[11]<0.5) then
                erro = 4
           
            
            elseif (data0[11]>0.5 and data1[11]>0.5 and data2[11]>0.5 and data3[11]<0.5 and data4[11]<0.5) then
                erro = 3
            
            
            elseif (data0[11]>0.5 and data1[11]>0.5 and data2[11]>0.5 and data3[11]<0.5 and data4[11]>0.5) then
                erro = 2
            
            
            elseif (data0[11]>0.5 and data1[11]>0.5 and data2[11]<0.5 and data3[11]<0.5 and data4[11]>0.5) then
                erro = 1
           
            
            elseif (data0[11]>0.5 and data1[11]>0.5 and data2[11]<0.5 and data3[11]>0.5 and data4[11]>0.5) then
                erro = 0
            
            
            elseif (data0[11]>0.5 and data1[11]<0.5 and data2[11]<0.5 and data3[11]>0.5 and data4[11]>0.5) then
                erro = -1
            
            
            elseif (data0[11]>0.5 and data1[11]<0.5 and data2[11]>0.5 and data3[11]>0.5 and data4[11]>0.5) then
                erro = -2
            
            
            elseif (data0[11]<0.5 and data1[11]<0.5 and data2[11]>0.5 and data3[11]>0.5 and data4[11]>0.5) then
                erro = -3
            
            
            elseif (data0[11]<0.5 and data1[11]>0.5 and data2[11]>0.5 and data3[11]>.5 and data4[11]>0.5) then
                erro = -4
            
            
            elseif (data0[11]>0.5 and data1[11]>0.5 and data2[11]>0.5 and data3[11]>.5 and data4[11]>0.5) then
                if (erroanterior <= -4) then
                    erro = -5
                
                else
                    erro = 5
                end
    end
            
end
    
    ------ FUNCAO DO CALCULO PID COM CONSTANTES KP, KI, KD
    function pid()
        TermoP = erro * kp
        
        integral = integral + erro
        TermoI = ki * integral
        
        derivativo = erro - erroanterior
        TermoD = derivativo * kd
        
        saida = TermoP + TermoI + TermoD
        
        erroanterior = erro
    end
    
    -------- ALTERA A VELOCIDADE DOS MOTORES COM A SAIDA DO SINAL DE ERRO DO CONTROLADOR PID
    
    function velocidade()
        
        sim.setJointTargetVelocity(motorEsquerdo, vMotor + saida)
        sim.setJointTargetVelocity(motorDireito, vMotor - saida)
       
    end
    
end

function sysCall_actuation()
    
end

function sysCall_sensing()
    -- LEITURA DOS SENSORES E SUAS ATRIBUIÇOES AS VARIAVEIS
    result0,data0=sim.readVisionSensor(sensor0)
    result1,data1=sim.readVisionSensor(sensor1)
    result2,data2=sim.readVisionSensor(sensor2)
    result3,data3=sim.readVisionSensor(sensor3)
    result4,data4=sim.readVisionSensor(sensor4)
    resultc,datac=sim.readVisionSensor(camera)
    distance = datac[15]

     -- SE SENSOR DO MEIO (2) DETECTA PRETO, ENTAO SEGUIR EM LINHA RETA
    if data2[11]<0.5 then
    	cruzamento=false
    	vMotor=20
    end
    

    -- SE O SEGUIDOR DE LINHA NAO ESTA EM CRUZAMENTO, EXECUTAR MUDANÇA DE TRAGETORIA COM PID
    if not cruzamento then
    	erroCalc()
    	pid()
    	velocidade()
    end
    
	-- SE SEGUIDOR DE LINHA ESTIVER EM CRUZAMENTO, ZERA O ERRO PARA EVITAR CORREÇOES INDESEJADAS
	-- CONTINUA CHAMANDO A FUNÇAO VOLTA 
    if cruzamento==true then
        erro=0
        integral = 0
        derivativo = 0
        virar(contadorVolta)
    end
    
   
	-- RECONHECE CORES A PARTIR DE UMA DISTANCIA MINIMA 
   if distance ~= nil then 	-- evitar comparaçao com valor nil (nulo)
      
        if distance < 0.037 then 	-- distancia
           
            if datac[14] > 0.4 then	-- se media da cor azul for significativa
            cruzamento=true
              
                if auxiliar==true then
                cruzamento=true
                    vMotor=0
                    contadorVolta=contadorVolta+1  -- para melhor controle de qual volta esta
                    auxiliar=false
                    
                    
                end
            end
            
            if datac[12] > 0.3 then	-- detecçao de vermelho
           
                
                auxiliar=true
                if auxTempo==true then
                    tAntes=sim.getSimulationTime()
                    auxTempo=false
                end
                print('tempo ', sim.getSimulationTime()-tAntes)
               
                if (sim.getSimulationTime()-tAntes<5.0) then	-- controle de tempo (5 segundos parado)
                    vMotor=0
                    sim.setJointTargetVelocity(motorEsquerdo, vMotor)
                    sim.setJointTargetVelocity(motorDireito, vMotor)
                    parar=true
                    erro=0
                    integral = 0
                    derivativo = 0
                        
                else
                    parar=false
                    vMotor=20
                    erroCalc()
                    pid()
                    velocidade()
                    
                end
            
            end
        end
   end
    
end

function sysCall_cleanup()
  
end