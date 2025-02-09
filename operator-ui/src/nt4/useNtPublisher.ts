import {Dispatch, MutableRefObject, SetStateAction, useEffect, useRef, useState} from "react";
import {NT4_Client, NTType} from "./NT4.ts";

export function useNtPublisher<T>(
    client: NT4_Client,
    topic: string,
    type: NTType,
    initialValue: T
): [MutableRefObject<T>, Dispatch<SetStateAction<T>>] {
    const [rawValue, setRawValue] = useState(initialValue)
    const ref = useRef(initialValue)

    useEffect(() => {
        ref.current = rawValue
    }, [rawValue]);
    useEffect(() => {
        client.publishTopic(topic, type)
        setInterval(() => {
            client.addSample(topic, ref.current)
        }, 20)
    }, []);

    return [ref, setRawValue]
}