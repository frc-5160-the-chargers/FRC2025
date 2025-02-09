import {useEffect, useState} from 'react'
import {NT4_Client} from "./nt4/NT4.ts";
import {useNtPublisher} from "./nt4/useNtPublisher.ts";
import {Button, Image, Text} from "@mantine/core";
import hexagonNodeSrc from "assets/node-image.png";

enum NTClientState {
    CONNECTING, READY
}

const IS_SIM = true

export function App() {
    const [clientState, setClientState] = useState(NTClientState.CONNECTING)
    const [client] = useState(
        new NT4_Client(
            IS_SIM ? "127.0.0.1" : "10.51.60.2",
            "OperatorUI",
            () => {},
            () => {},
            () => {},
            () => setClientState(NTClientState.READY),
            () => setClientState(NTClientState.CONNECTING)
        )
    )

    const [, setRawValue] = useNtPublisher(client, "test/test2", "boolean", false)

    useEffect(() => {
        client.connect().then(() => console.log("Client connected"))
        addEventListener("pagehide", () => client.disconnect())
    }, []);

    if (clientState == NTClientState.READY) {
        return (
            <Text>
                Ready!
                <Image src={hexagonNodeSrc}></Image>
                <Button onClick={() => setRawValue(true)}>Test</Button>
            </Text>
        )
    } else {
        return (
            <Text>
                Loading...
            </Text>
        )
    }
}
