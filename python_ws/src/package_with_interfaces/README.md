# Interfaces: Messages - Services - Actions - Nested Messages
All messages are defined in the msg folder while services are defined in the srv folder<br>

> To use a message in same package in another file, simply use the filename as message type
> To use a message from another package, use <packagename>/<filename> as message type

A topic only accepts one message type and the topic is uniquely identified by a string.

#### EXPLANATION OF DIRECTORY AND FILE STRUCTURE
- Messeges are created in a file that ends with `.msg` and they are saved in a `msg` directory.
Same rule applies to services and actions, both files ending with `.srv` and `.action` and saved in
`srv` and `action` directory respectively.
- In services and actions, a Request is seperated from a response using `---`<br>
