package org.firstinspires.ftc.teamcode.drive.ComputerVision;

import java.io.*;
import java.net.*;
import java.util.concurrent.atomic.AtomicReference;

public class CameraServer implements Runnable {
    private final int port;
    private ServerSocket serverSocket;
    private boolean running = true;
    private final AtomicReference<byte[]> frame = new AtomicReference<>();

    public CameraServer(int port) {
        this.port = port;
    }

    public void updateFrame(byte[] jpegData) {
        frame.set(jpegData);
    }

    @Override
    public void run() {
        try {
            serverSocket = new ServerSocket(port);

            while (running) {
                Socket client = serverSocket.accept();
                new Thread(() -> handleClient(client)).start();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void handleClient(Socket client) {
        try (OutputStream os = client.getOutputStream();
             BufferedReader br = new BufferedReader(new InputStreamReader(client.getInputStream()))) {

            while (br.ready()) {
                br.readLine();
            }

            String header =
                    "HTTP/1.1 200 OK\r\n" +
                            "Connection: close\r\n" +
                            "Cache-Control: no-cache\r\n" +
                            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";

            os.write(header.getBytes());
            os.flush();

            while (running && !client.isClosed()) {
                byte[] currentFrame = frame.get();
                if (currentFrame != null) {
                    String frameHeader =
                            "--frame\r\n" +
                                    "Content-Type: image/jpeg\r\n" +
                                    "Content-Length: " + currentFrame.length + "\r\n\r\n";

                    os.write(frameHeader.getBytes());
                    os.write(currentFrame);
                    os.write("\r\n".getBytes());
                    os.flush();
                }
                Thread.sleep(20);
            }

        } catch (Exception e) {
            System.out.println("Client error: " + e.getMessage());
        } finally {
            try { client.close(); } catch (IOException ignored) {}
        }
    }

    public void stop() {
        running = false;
        try { serverSocket.close(); } catch (IOException ignored) {}
    }
}