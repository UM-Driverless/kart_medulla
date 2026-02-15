- Libreria km_coms
Hacer funcion para que formatee el payload en funcion del tipo que se le ponga, 
y luego ya se le pase  el payload y todo a la funcion send, y luego hacer la 
funcion inversa para cuando se recibe

(Recibir un mensaje)
Buffer Uart --> Buffer libreria --> (se formatea mensaje y se pone en cola) --> se formatea payload en funcion del tipo de mensaje

(Enviar un mensaje)
Se formatea payload en funcion del tipo de mensaje --> (buffer Uart)

- main
Montar toda la estructura

- libreria maquina de estados
Hacerla entera

- Libreria para guardar cosas
Se puede hacer una libreria que cree 'objetos' y que estos queden guardados hay
asi cuando se quiere info sobre algo solo hay que pedir el objeto

- Libreria gamc
Hacer la libreria para controlar los mandos

- Mover todo lo relacionado con pines a libreria gpio???

- Revisar si se puede poner la libreria de dac y ledc mas modernas ahora que esta todo configurado