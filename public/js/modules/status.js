export class Status {
    constructor(icon_element, message_element) {
        this.icon = icon_element;
        this.message = message_element;

        this._pendingStatus = null;
        this._pendingMessage = null;
        this._lastStatus = null;
        this._lastMessage = null;

        this._updateInterval = setInterval(() => {
            if (!this.icon == null || !this.icon.isConnected || this.message == null || !this.message.isConnected) {
                clearInterval(this._updateInterval);
                return;
            }

            if (this._pendingStatus !== this._lastStatus || this._pendingMessage !== this._lastMessage) {
                this._applyStatus(this._pendingStatus, this._pendingMessage);
                this._lastStatus = this._pendingStatus;
                this._lastMessage = this._pendingMessage;
            }
        }, 200);
    }

    _applyStatus(status, message) {
        // Reset classes
        this.icon.classList.remove("icon-error", "icon-warn");
        this.message.classList.remove("status-error", "status-warn");

        // Apply new state
        switch (status) {
            case "ok":
                this.message.innerText = "Status: " + (message || "Ok");
                break;
            case "warn":
                this.icon.classList.add("icon-warn");
                this.message.classList.add("status-warn");
                this.message.innerText = "Status: " + message;
                break;
            case "error":
                this.icon.classList.add("icon-error");
                this.message.classList.add("status-error");
                this.message.innerText = "Status: " + message;
                break;
        }
    }

    async setOK(message) {
        this._pendingStatus = "ok";
        this._pendingMessage = message;
    }

    async setWarn(message) {
        this._pendingStatus = "warn";
        this._pendingMessage = message;
    }

    async setError(message) {
        this._pendingStatus = "error";
        this._pendingMessage = message;
    }
}
