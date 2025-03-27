package com.example.myapplication.Socketconnection;

import android.os.Handler;
import android.util.Log;

import com.example.myapplication.MainActivity;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;

public class TCPClient {

    private String serverMessage;
    public static final String SERVERIP = "192.168.1.118";  //192.168.1.118"; //your computer IP address
    public static final int SERVERPORT = 8080;
    private OnMessageReceived mMessageListener = null;
    private boolean mRun = false;
    private String ipAddressVar;
    private int ipPortVar;
    private MainActivity owner;
    private TCPClientOwner sessionOwner;

    PrintWriter out;
    BufferedReader in;

    public static TCPClient singleton;

    /**
     *  Constructor of the class. OnMessagedReceived listens for the messages received from server
     */
    public TCPClient(OnMessageReceived listener, MainActivity owner) {
        mMessageListener = listener;
        ipAddressVar = null;
        this.owner = owner;
        this.sessionOwner = null;
    }

    public void setSessionOwner(TCPClientOwner sessionOwner){

        this.sessionOwner = sessionOwner;
    }

    /**
     * Sends the message entered by client to the server
     * @param message text entered by client
     */
    public void sendMessage(String message){
        if (out != null && !out.checkError()) {
            if (sessionOwner != null) {
                Handler buttonHandler = new Handler(sessionOwner.getMainLooper());
                Runnable myRunnable = new Runnable() {
                    @Override
                    public void run() {
                        sessionOwner.disableButtons();
                    }
                };
                buttonHandler.post(myRunnable);
            }
            out.println(message);
            out.flush();
        }
    }

    public void stopClient(){

        mRun = false;
    }

    public void run() {

        mRun = true;

        try {
            //here you must put your computer's IP address.
            InetAddress serverAddr;

            if (ipAddressVar == null) {
                serverAddr = InetAddress.getByName(SERVERIP);
                ipPortVar = SERVERPORT;
            }
            else {
                serverAddr = InetAddress.getByName(ipAddressVar);
                //use ipPortVar that was input by user
            }

            Log.e("TCP Client", "C: Connecting...");

            //create a socket to make the connection with the server
            Socket socket = new Socket(serverAddr, ipPortVar);
            //owner.connected();
            Log.e("TCP Client", "C: Connected");
            Handler mainHandler = new Handler(owner.getMainLooper());
            Runnable myRunnable = new Runnable(){
                @Override
                public void run() {
                    owner.connected();
                }
            };
            mainHandler.post(myRunnable);
            try {

                //send the message to the server
                out = new PrintWriter(new BufferedWriter(new OutputStreamWriter(socket.getOutputStream())), true);

                out.print("Session started!\r\n");
                out.flush();

                Log.e("TCP Client", "C: Sent.");

                Log.e("TCP Client", "C: Done.");


                //receive the message which the server sends back
                in = new BufferedReader(new InputStreamReader(socket.getInputStream()));


                //in this while the client listens for the messages sent by the server
                while (mRun) {
                    Log.e("TCP Client", "Running");
                    serverMessage = in.readLine();
                    Log.e("TCP Client", "Receiving" + in);


                    if (serverMessage != null && mMessageListener != null) {
                        //call the method messageReceived from MyActivity class
                        mMessageListener.messageReceived(serverMessage);
                        Handler buttonHandler = new Handler(sessionOwner.getMainLooper());
                        //Log.d("TCP Client", "serverMessage:" + serverMessage);

                        Runnable myRunnable2 = new Runnable(){
                            private String serverMsg = serverMessage;
                            @Override
                            public void run() {
                                Log.e("TCP Client", "serverMessage TIME 4 is:" + serverMessage);
                                Log.e("TCP Client", "within class serverMsg is:" + serverMsg);
                                if (serverMessage == null){
                                    sessionOwner.messageReceived(serverMsg);
                                    Log.e("TCP Client", "using serverMsg:" + serverMsg);
                                }
                                else if (!serverMessage.equals(serverMsg)){ //aditi: Should handle the case when SHOWSTRUCTURE messages are getting lost
                                    sessionOwner.messageReceived(serverMsg);
                                    Log.e("TCP Client", "THE TWO PRINTED MESSAGES ARE NOT THE SAME, using serverMsg: " + serverMsg);
                                }
                                else {
                                    sessionOwner.messageReceived(serverMessage);
                                    Log.e("TCP Client", "using serverMessage:" + serverMessage);
                                }
                            }
                        };
                        buttonHandler.post(myRunnable2);
                    }
                    serverMessage = null;

                }


                Log.e("RESPONSE FROM SERVER", "S: Received Message: '" + serverMessage + "'");


            } catch (Exception e) {

                Log.e("TCP", "S: Error", e);

            } finally {
                //the socket must be closed. It is not possible to reconnect to this socket
                // after it is closed, which means a new socket instance has to be created.
                socket.close();
            }

        } catch (Exception e) {

            Log.e("TCP", "C: Error", e);

        }

    }

    public String getIpAddress() {

        return ipAddressVar;
    }

    public void setIpAddress(String ipAddressVar) {

        this.ipAddressVar = ipAddressVar;
    }

    public int getIpPortVar() {

        return ipPortVar;
    }

    public void setIpPortVar(int ipPortVar){

        this.ipPortVar = ipPortVar;
    }

    public interface OnMessageReceived {
        public void messageReceived(String message);
    }
}
