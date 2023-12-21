Antenne conectée en LoRa

Presentation générale :
Ce projet a pour but de faire communiquer deux modules avec des antennes utilisant le protocole LoRa et de permettre aux agriculteurs de connaitre des données relevées dans leurs champ.
Actuellement les modules sont capables de relever la température et l'humidité de l'air ainsi que l'humidité du sol et de transmettre ces données sur une distance de 4km (test fait en zone urbaine)

Présentation du matériel electronique :

Module émetteur : 
    ESP32
    module LoRa Ebyte E32 433
    Sonde d'humidité et de température de l'air DHT 22
    Pile CR123A pour la mise en sommeil.
    Batterie 3s LiOn pour l'alimentation générale du module
    Sonde d'humidité du sol HD38
    régulateur de tension 5V


Module Récepteur : 
    ESP32
    module LoRa Ebyte E32 433
    Ecran TFT 4 lignes
    Alimentation sur secteur par un chargeur de téléphone délivrant du 5V
    
