let wateringFlag = true;

/**
 * @function sendMessage
 * Effectue une requete HTTP une commande de type XML composee d'une action
 * @param {*} spec : Type d'action a realiser
 * @param {*} action : Message a envoyer par requete
 */
function sendMessage(spec, action) {
    let xhttp = new XMLHttpRequest();
    let url = "/" + spec + "?action=" + action; // Construire l'URL avec le paramètre action
    xhttp.open("GET", url, true);
    xhttp.send();
}

/**
 * @function robRight
 * Commandes directionnelle vehicule : droite
 * Moteurs droits en marche avant et moteurs gauches en marche arrieres
 */
function robRight() { sendMessage("rob_ctrl", "right"); }

/**
 * @function robLeft
 * Commandes de direction pour le vehicule : gauche
 * Moteurs gauches en marche avant et moteurs droits en marche arrieres
 */
function robLeft() { sendMessage("rob_ctrl", "left"); }

/**
 * @function robForward
 * Commandes de direction pour le vehicule : avant
 * Moteurs droits et gauches en marche avant
 */
function robForward() { sendMessage("rob_ctrl", "forward"); }

/**
 * @function robBackward
 * Commandes de direction pour le vehicule : arriere
 * Moteurs droits et gauches en marche arriere
 */
function robBackward() { sendMessage("rob_ctrl", "backward"); }

/**
 * @function robRightForward
 * Commandes de direction pour le vehicule : avant droite
 * Moteurs droits et gauches en marche avant, moteurs droits ralentis
 */
function robRightForward() { sendMessage("rob_ctrl", "right_forward"); }

/**
 * @function robLeftForward
 * Commandes de direction pour le vehicule : avant gauche
 * Moteurs droits et gauches en marche avant, moteurs gauches ralentis
 */
function robLeftForward() { sendMessage("rob_ctrl", "left_forward"); }

/**
 * @function robRightBackward
 * Commandes de direction pour le vehicule : arriere droite
 * Moteurs droits et gauches en marche arriere, moteurs droits ralentis
 */
function robRightBackward() { sendMessage("rob_ctrl", "right_backward"); }

/**
 * @function robLeftBackward
 * Commandes de direction pour le vehicule : arriere gauche
 * Moteurs droits et gauches en marche arriere, moteurs gauches ralentis
 */
function robLeftBackward() { sendMessage("rob_ctrl", "left_backward"); }

/**
 * @function robStop
 * Commandes d'arret des moteurs du vehicule
 */
function robStop() { sendMessage("rob_ctrl", "stop"); }

/**
 * @function fetchData 
 * Effectue une requete en HTTP pour transmettre des donnees en JSON
 * Cela permet de faire transister pluiseurs donnees en meme temps
 * @param {*} elementId : Element html a modifier
 * @param {*} url : Section de lien auquel effectuer la modification
 * @param {*} time : Temps de rafraichissement en millisecondes
 */ 
function fetchData(elementId, url, time) {
    function getData(){
        let xhr = new XMLHttpRequest();
        xhr.onreadystatechange = function() {
            // Verification de l'etat de succes de la requete
            if (xhr.readyState == 4 && xhr.status == 200) {
                let data = JSON.parse(xhr.responseText);
                let html = ""; // Variable pour stocker le HTML genere
                for (let key in data) { // Iterer a travers les proprietes de l'objet JSON
                    // Ajouter le nom de la propriete et sa valeur a la variable HTML
                    if (data.hasOwnProperty(key))
                        html += `<b>${key} :</b> ${data[key]}<br>`;
                }
                // Afficher le HTML genere dans l'element avec l'ID specifie
                document.getElementById(elementId).innerHTML = html;
            }
        };
        // Configure la requete HTTP GET avec l'URL du point de terminaison
        xhr.open("GET", url, true);
        xhr.send(); // Envoie la requete
    }

    // Execution une fois apres le delai
    setTimeout(getData, 500);
    // Execution en boucle selon le delai
    if(time != 0)
        setInterval(getData, time);
}

/**
 * @function getPlainData
 * Effectue une requete en HTTP pour transmettre des donnees non formatees
 * @param {*} elementId : Element html a modifier
 * @param {*} url : Section de lien auquel effectuer la modification
 * @param {*} time : Temps de rafraichissement en millisecondes
 */ 
function getPlainData(elementId, url, time){
    function getData(){
        let xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function(){
            if(this.readyState == 4 && this.status == 200)
                document.getElementById(elementId).innerHTML = this.responseText;
        };
        xhttp.open("GET", url, true);
        xhttp.send();
    }
    // Execution une fois apres le delai
    setTimeout(getData, 500);
    // Execution en boucle selon le delai
    if(time != 0)
        setInterval(getData, time); 
}

/**
 * @function activePump
 * Permet d'activer ou de desactiver la pompe
 */
function activePump() { 
    if (wateringFlag) {
        document.getElementById("vat").style.border = "solid #626e1b 3px";
        sendMessage("vat_ctrl", "on");
        wateringFlag = false;
    } else {
        document.getElementById("vat").style.border = "solid #8a8 3px";
        sendMessage("vat_ctrl", "off");
        wateringFlag = true;
    }
    getPlainData("data_vat", "rd_vat", 0);
}

/**
 * @function tarePump
 * Reinitialiser l'estimation de la cuve
 */
function tarePump() { 
    if (wateringFlag) {
        document.getElementById("tare").style.border = "solid #8a8 3px";
        sendMessage("vat_ctrl", "tare");
    } else {
        document.getElementById("tare").style.border = "solid #a00 3px"; 
    }
    getPlainData("data_vat", "rd_vat", 0);
}

/**
 * @function getFeedback
 * Renvoyer des informations sur les servos-moteurs du bras robotique
 */
function getFeedback(){
    let jsonCmd = { "T": 105 }
    let jsonString = JSON.stringify(jsonCmd);
    fetchData("data_arm", "arm_ctrl?action=" + jsonString, 0);
}

/**
 * @function cmdInit
 * Retablir la position initiale du bras robotique
 */
function cmdInit() {
    let jsonCmd = {
        "T":102, "base":0, "shoulder":-1.57, "elbow":3.10,
        "hand":3.20, "spd":0, "acc":0
    }

    let jsonString = JSON.stringify(jsonCmd);
    sendMessage("arm_ctrl", jsonString);
    getFeedback();
}

/**
 * @function cmdSend
 * Permet d'interagir avec un servo-moteur du bras robotique
 * @param {*} inputT Commande a traiter pour un moteur
 * @param {*} inputA Axe de rotation d'un moteur
 * @param {*} inputB Deplacement moteur
 */
function cmdSend(inputT, inputA, inputB) {
    let jsonCmd = {
        "T": 123, "m": inputT, "axis": inputA,
        "cmd": inputB, "spd": 10
    }
    let jsonString = JSON.stringify(jsonCmd);
    sendMessage("arm_ctrl", jsonString);
    getFeedback();
}

/**
 * @function cmdStop
 * Fonction d'arret d'urgence de tous les servo-moteurs du bras robotique
 */
function cmdStop() {
    cmdSend(0,1,0);
    cmdSend(0,2,0);
    cmdSend(0,3,0);
    cmdSend(0,4,0);
}

fetchData("data_dyn", "rd_dyn", 60000);
fetchData("data_sta", "rd_sta", 0);
