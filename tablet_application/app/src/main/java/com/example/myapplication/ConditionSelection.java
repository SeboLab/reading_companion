package com.example.myapplication;


import android.annotation.SuppressLint;
import android.graphics.Color;
import android.os.AsyncTask;
import android.os.Bundle;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import com.example.myapplication.Socketconnection.TCPClient;
import com.example.myapplication.Socketconnection.TCPClientOwner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class ConditionSelection extends AppCompatActivity implements TCPClientOwner, View.OnClickListener{

    private Button Human;
    private Button Robot;
    private Spinner spinnerMenu;
    private Button start;


    @SuppressLint({"WrongViewCast", "MissingInflatedId"})
    protected void onCreate(Bundle savedInstanceState) {

        // transferring ownership to this activity page
        if (TCPClient.singleton != null) {
            TCPClient.singleton.setSessionOwner(this);
        }
        Global.getInstance().setstorynumber(1);

        super.onCreate(savedInstanceState);
        setContentView(R.layout.conditionselection);

        Human = findViewById(R.id.Human);
        Human.setOnClickListener(this);
        Robot = findViewById(R.id.Robot);
        Robot.setOnClickListener(this);
        start = findViewById(R.id.finish);
        start.setOnClickListener(this);

        List<String> spinnerItems = Arrays.asList("Pick a Story!", getString(R.string.M), getString(R.string.N), getString(R.string.O), getString(R.string.P), getString(R.string.Q));
        boolean[] selectedOptions = new boolean[spinnerItems.size()];
        spinnerMenu = findViewById(R.id.spinnerMenu);
        ArrayAdapter<String> spinnerAdapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, spinnerItems);

        // Specify the layout for dropdown items
        spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);

        // Set the adapter on the spinner
        spinnerMenu.setAdapter(spinnerAdapter);
        spinnerMenu.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                // Update the selectedOptions array based on the selected item
                selectedOptions[position] = !selectedOptions[position];
                List<String> selectedOptionList = new ArrayList<>();
                for (int i = 0; i < selectedOptions.length; i++) {
                    if (selectedOptions[i]) {
                        selectedOptionList.add(spinnerItems.get(i));
                    }
                }
                selectedOptionList.remove(0);
                String[] selectedOptionsArray = selectedOptionList.toArray(new String[selectedOptionList.size()]);

                // Display the selected options
                Toast.makeText(ConditionSelection.this, "Selected options: " + selectedOptionList, Toast.LENGTH_LONG).show();
                Global.getInstance().setStoryselected(selectedOptionsArray);
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                // Handle when no item is selected
            }
        });

    }

    /* checking whether robot or human button was selected, and either letting the
        raspberry pi pick the stories or opening up our story selection menu 
        respectively */
    public void onClick(View v) {

        if (v.getId() == R.id.Robot) {
            Global.getInstance().setMode(0);
            String condition_selected = "condition;0;";
            Robot.setClickable(false);
            Robot.setClickable(true);
            new Thread(() -> {
                if (TCPClient.singleton != null) {
                    TCPClient.singleton.sendMessage(condition_selected);
                }
            }).start();
        }
        if (v.getId() == R.id.Human) {
            Global.getInstance().setMode(1);
            String condition_selected = "condition;1;";
            new Thread(() -> {
                if (TCPClient.singleton != null) {
                    TCPClient.singleton.sendMessage(condition_selected);
                }
            }).start();
            if (spinnerMenu.getVisibility() == View.VISIBLE) {
                Human.setBackgroundColor(Color.LTGRAY);
                spinnerMenu.setVisibility(View.INVISIBLE);
                start.setVisibility(View.INVISIBLE);
            } else {
                Human.setBackgroundColor(Color.GRAY);
                spinnerMenu.setVisibility(View.VISIBLE);
                start.setVisibility(View.VISIBLE);
            }
        }
        if (v.getId() == R.id.finish){
            String firststory = Global.getInstance().getStoryselected()[0];
            Global.startHumanCondition(ConditionSelection.this, firststory);
        }
    }

    //functions for enabling talking with raspberry pi:

    @Override
    public void messageReceived(String message) {
        System.out.println("Message received");
        if (message==null){
            TCPClient.singleton.sendMessage("no message received from server. please pick a story");
        }
        else {
            List<String> stories_to_read = Arrays.asList(message.split(";"));
            if (stories_to_read.get(0).equals("Return connection messagestory_selection")) {
                String string_story_indices = stories_to_read.get(1);
                string_story_indices = string_story_indices.substring(1, string_story_indices.length() - 1);
                Global.getInstance().setStoryselected(string_story_indices.split(","));
                String storyone = Global.getInstance().getStoryselected()[0];
                Global.startRobotCondition(ConditionSelection.this, storyone);
            }
        }
    }

    @Override
    public void disableButtons() {

    }

    @SuppressWarnings("deprecation")
    public class sendMessage extends AsyncTask<String, Void, Void> {
        @SuppressWarnings("deprecation")
        @Override
        protected Void doInBackground(String... message) {
            String message1 = message[0];
            if (TCPClient.singleton != null) {
                TCPClient.singleton.sendMessage(message1);
            }
            return null;
        }
    }
}

