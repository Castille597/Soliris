const httpBase = 'http://192.168.1.26:5050';
const wsBase = 'ws://192.168.1.26:5050';
String wsUrlFor(String userId) => '$wsBase/ws?userId=$userId';
