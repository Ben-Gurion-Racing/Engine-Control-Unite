Documtaion 15/6/2019

- There is the ability to send and reacive UDP measage , to reacive a message the sender should have both ip and port match as set in the udp connect function 
- to pull the meassage it shuld call the   ethernetif_input(&gnetif) function ;
- we started to impliment the reaciving message handeling 
- we want to set a send measseage from the interrupt handler 

