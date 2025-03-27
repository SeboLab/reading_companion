package com.example.myapplication.Pages;

import android.content.Intent;
import android.graphics.Color;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.text.TextUtils;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.LinearLayoutCompat;

import com.example.myapplication.ConditionSelection;
import com.example.myapplication.Global;
import com.example.myapplication.R;
import com.example.myapplication.Socketconnection.TCPClient;
import com.example.myapplication.Socketconnection.TCPClientOwner;

import java.util.ArrayList;
import java.util.List;

//same as first page! again another brute force way to get it working except for lines 175-177 where it
// makes the code go back to condition selection if its the second story and otherwise
// starts at first page again!
public class ThirdPage extends AppCompatActivity implements TCPClientOwner {
    LinearLayoutCompat buttonContainer;
    private Button nextPageButton;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (TCPClient.singleton != null) {
            TCPClient.singleton.setSessionOwner(this);
        }

        setContentView(R.layout.page_layout);
        buttonContainer = findViewById(R.id.buttonContainer);
        buttonContainer.setOrientation(LinearLayoutCompat.VERTICAL);
        buttonContainer.setGravity(Gravity.CENTER_HORIZONTAL);
        List<Button> wordsselected = new ArrayList<>();
        String paragraph = getString(Global.getInstance().getWords3());
        String[] words = paragraph.split(" ");
        LinearLayout.LayoutParams layoutParams;
        LinearLayout currentLinearLayout = new LinearLayout(this);
        currentLinearLayout.setOrientation(LinearLayout.HORIZONTAL);

        int screenWidth = 1850;
        int rowwidth = 100;
        for (int i = 0; i < words.length; i++) {
            if (rowwidth >= screenWidth - 150 && i > 0) {
                rowwidth = 100;
                buttonContainer.addView(currentLinearLayout);
                currentLinearLayout = new LinearLayout(this);
                currentLinearLayout.setOrientation(LinearLayout.HORIZONTAL);
            }

            Button button = new Button(this);
            button.setText(words[i]);

            int width;
            int height;

            if (getString(Global.getInstance().getStory()).equals("Fizzy Water") ||  getString(Global.getInstance().getStory()).equals("Government")){
                button.setTextSize(Global.specialButtonFontSize);
                height = Global.specialHeight;
                if (words[i].length() <= 3){
                    width = words[i].length() * 25 + 10;
                } else if (words[i].length() <= 6){
                    width = words[i].length() * 21 + 10;
                } else if (words[i].length() <= 10){
                    width = words[i].length() * 19 + 10;
                } else {
                    width = words[i].length() * 18;
                }
            } else{
                button.setTextSize(Global.buttonFontSize);
                height = Global.height;
                if (words[i].length() <= 3){
                    width = words[i].length() * 26 + 10;
                } else if (words[i].length() <= 6){
                    width = words[i].length() * 22 + 10;
                } else if (words[i].length() <= 10){
                    width = words[i].length() * 20 + 10;
                } else {
                    width = words[i].length() * 19;
                }
            }

            rowwidth += width;
            button.setLayoutParams (new LinearLayoutCompat.LayoutParams(width, height));
            button.setAllCaps(false);
            button.setId(i);
            button.setBackgroundColor(Color.WHITE);
            button.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    if (wordsselected.contains(button)) {
                        wordsselected.remove(button);
                        button.setBackgroundColor(Color.WHITE);
                    } else {
                        wordsselected.add(button);
                    }
                }
            });
            currentLinearLayout.addView(button);
        }
        buttonContainer.addView(currentLinearLayout);
        Button highlightButton = new Button(this);
        highlightButton.setBackgroundColor(Color.rgb(230, 230, 250));
        highlightButton.setText("Done");
        highlightButton.setLayoutParams(new LinearLayoutCompat.LayoutParams(LinearLayoutCompat.LayoutParams.WRAP_CONTENT, LinearLayoutCompat.LayoutParams.WRAP_CONTENT));
        highlightButton.setGravity(Gravity.CENTER);
        highlightButton.setOnClickListener(v -> {
            highlightButton.setClickable(false);
            highlightButton.setVisibility(View.INVISIBLE);
            nextPageButton.setClickable(false);
            nextPageButton.setBackgroundColor(Color.GRAY);
            nextPageButton.setVisibility(View.VISIBLE);ArrayList<Integer> words_indice = new ArrayList<>();
            for (int i = 0; i < wordsselected.size(); i++){
                Integer index = wordsselected.get(i).getId();
                words_indice.add(index);
            }
            String buttonIdsString = TextUtils.join(",", words_indice) + ";";
            // if (Global.getInstance().getMode().equals(1)){
            new Thread(() -> {
                if (TCPClient.singleton != null) {
                    System.out.println("message being sent");
                    TCPClient.singleton.sendMessage("word_selections;"+buttonIdsString);
                    System.out.println("word_selections;"+buttonIdsString);
                }
            }).start();
            // }
            new Handler().postDelayed(() -> {
                nextPageButton.setClickable(true);
            }, 5000);



        });
        buttonContainer.addView(highlightButton);
        nextPageButton = new Button(this);
        nextPageButton.setText("New Story");
        nextPageButton.setLayoutParams(new LinearLayoutCompat.LayoutParams(
                LinearLayoutCompat.LayoutParams.WRAP_CONTENT, LinearLayoutCompat.LayoutParams.WRAP_CONTENT));
        nextPageButton.setGravity(Gravity.CENTER);
        nextPageButton.setVisibility(View.INVISIBLE);
        nextPageButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // Start the next activity here
                new Thread(() -> {
                    if (TCPClient.singleton != null) {
                        TCPClient.singleton.sendMessage("next page;");
                    }
                }).start();
                if (Global.getInstance().getstorynumber().equals(1)) {
                    if (Global.getInstance().getMode().equals(1)) {
                        String secondstory = Global.getInstance().getStoryselected()[1];
                        Global.getInstance().setstorynumber(2);
                        Global.startHumanCondition(ThirdPage.this, secondstory);
                    }
                    else {
                        String secondstorystr = Global.getInstance().getStoryselected()[1];
                        String secondstory = secondstorystr.trim();
                        Global.getInstance().setstorynumber(2);
                        Global.startRobotCondition(ThirdPage.this, secondstory);
                    }
                }
                else if (Global.getInstance().getstorynumber().equals(2)){
                    Intent intent = new Intent(ThirdPage.this, ConditionSelection.class);
                    startActivity(intent);
                }
            }
        });
        buttonContainer.addView(nextPageButton);
    }

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


    @Override
    public void messageReceived(String message) {
        System.out.println("highlight"+message);
        Global.processMessage(message, buttonContainer);
    }

    @Override
    public void disableButtons() {

    }
}
