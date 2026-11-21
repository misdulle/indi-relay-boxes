"""In-memory view of the INDI property tree relevant to the relay boxes."""

from .indi import SWITCH_VECTOR

HIDDEN_RELAY_NAMES = ("none",)

_MEMBER_VALUE_KEYS = {
    "SwitchVector": ("on",),
    "TextVector": ("text",),
    "NumberVector": ("value", "min", "max", "step"),
    "LightVector": ("light",),
}


class IndiModel:
    def __init__(self):
        self.properties = {}
        self.devices = set()

    def apply_vector(self, vector):
        if not vector.get("device"):
            return
        self.devices.add(vector["device"])
        key = (vector["device"], vector["name"])
        current = self.properties.get(key)
        if current is not None and not vector.get("defined", True):
            # set*Vector updates carry state but no labels; merge them into
            # the stored definition instead of wiping names from PORTS etc.
            self.properties[key] = self._merge_update(current, vector)
        else:
            self.properties[key] = vector

    @staticmethod
    def _merge_update(current, vector):
        merged = dict(current)
        merged["defined"] = True
        merged["state"] = vector.get("state", current.get("state"))
        if vector.get("message"):
            merged["message"] = vector["message"]
        value_keys = _MEMBER_VALUE_KEYS.get(vector["type"], ())
        members = dict(current["members"])
        for name, member in vector["members"].items():
            updated = dict(members.get(name, member))
            updated.update({key: member[key] for key in value_keys if key in member})
            members[name] = updated
        merged["members"] = members
        return merged

    def remove(self, device, name):
        self.properties.pop((device, name), None)

    def get(self, device, name):
        return self.properties.get((device, name))

    def devices_with(self, prop_name, vector_type):
        found = []
        for (device, name), vector in self.properties.items():
            if name == prop_name and vector["type"] == vector_type:
                found.append(device)
        return sorted(found)

    def relay_devices(self):
        return self.devices_with("PORTS", SWITCH_VECTOR)

    def hidden_relay_names(self):
        return {n.lower() for n in HIDDEN_RELAY_NAMES}

    def ports(self, device):
        vector = self.get(device, "PORTS")
        if vector is None:
            return []
        hidden = self.hidden_relay_names()
        ports = []
        for member in vector["members"].values():
            label = member["label"] or member["name"]
            if label.strip().lower() in hidden:
                continue
            ports.append({
                "name": member["name"],
                "label": label,
                "on": member["on"],
                "state": vector["state"],
            })
        ports.sort(key=lambda p: p["name"])
        return ports

    def port_member(self, device, token):
        vector = self.get(device, "PORTS")
        if vector is None:
            return None
        members = vector["members"]
        if isinstance(token, int):
            return self._port_at_index(members, token)
        wanted = str(token).strip().lower()
        for member in members.values():
            if member["name"].lower() == wanted or (member["label"] or "").strip().lower() == wanted:
                return member["name"]
        try:
            return self._port_at_index(members, int(token))
        except ValueError:
            return None

    @staticmethod
    def _port_at_index(members, index):
        name = "PORT%02d" % index
        if name in members:
            return name
        names = sorted(members)
        return names[index - 1] if 0 < index <= len(names) else None

    def set_port_member_on(self, device, member_name, on):
        vector = self.get(device, "PORTS")
        if vector is None:
            return
        if member_name in vector["members"]:
            vector["members"][member_name]["on"] = on

    def roof_state(self, device):
        vector = self.get(device, "DOME_SHUTTER")
        if vector is None:
            return {"text": "offline", "busy": False, "state": None}
        open_on = vector["members"].get("SHUTTER_OPEN", {}).get("on", False)
        close_on = vector["members"].get("SHUTTER_CLOSE", {}).get("on", False)
        state = vector["state"]
        if state == "IPS_BUSY":
            return {"text": "moving (opening)" if open_on else "moving (closing)",
                    "busy": True, "state": "moving"}
        if open_on:
            return {"text": "open", "busy": False, "state": "open"}
        if close_on:
            return {"text": "closed", "busy": False, "state": "closed"}
        if state == "IPS_ALERT":
            return {"text": "error", "busy": False, "state": "error"}
        return {"text": "unknown", "busy": False, "state": "unknown"}

    def roof_safety(self, device):
        vector = self.get(device, "ROOF_SAFETY_STATUS")
        if vector is None:
            return {}
        return {member["name"]: member["light"] for member in vector["members"].values()}

    def port_names(self, device):
        vector = self.get(device, "PORT_NAMES")
        if vector is None:
            return {}
        return {m: t["text"] for m, t in vector["members"].items()}

    def host_settings(self, device):
        host = self.get(device, "BOX_HOST")
        net = self.get(device, "BOX_NETWORK")
        info = {}
        if host is not None:
            for member in host["members"].values():
                info[member["name"]] = member["text"]
        if net is not None:
            for member in net["members"].values():
                info[member["name"]] = "%.0f" % member["value"]
        return info

    def config_summary(self):
        lines = []
        for device in sorted(self.relay_devices()):
            lines.append("%s (relay box): %d controlled relays" % (device, len(self.ports(device))))
            info = self.host_settings(device)
            if info:
                lines.append("  host %s port %s ADDR %s" % (
                    info.get("HOST", "?"),
                    info.get("TCP_PORT", "?"),
                    info.get("ADDR", "n/a")))
            for port in self.ports(device):
                lines.append("  %s: %s" % (port["label"], "ON" if port["on"] else "off"))
        for device in self.devices_with("DOME_SHUTTER", SWITCH_VECTOR):
            lines.append("%s (roof): %s" % (device, self.roof_state(device)["text"]))
        return lines
