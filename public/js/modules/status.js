export class Status {

    //General helper class for setting up widget status a la rviz

    constructor(icon_element, message_element) {
        this.icon = icon_element;
        this.message = message_element;
    }

    async setOK(message){
        this.icon.classList.remove("icon-error");
        this.icon.classList.remove("icon-warn");

        this.message.classList.remove("status-error");
        this.message.classList.remove("status-warn"); 

        if(arguments.length == 1){
            this.message.innerText = "Status: "+message;
        }else{
            this.message.innerText = "Status: Ok";
        }        
    }

    async setWarn(message){
        this.icon.classList.remove("icon-error");
        this.icon.classList.add("icon-warn");

        this.message.classList.remove("status-error");
        this.message.classList.add("status-warn"); 

        this.message.innerText = "Status: "+message;

    }

    async setError(message){
        this.icon.classList.remove("icon-warn");
        this.icon.classList.add("icon-error");

        this.message.classList.remove("status-warn");
        this.message.classList.add("status-error"); 
        this.message.innerText = "Status: "+message;
    }

}