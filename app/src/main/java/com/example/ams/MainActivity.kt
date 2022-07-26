package com.example.ams

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.view.Menu
import android.view.MenuInflater
import android.view.MenuItem
import android.widget.Toast

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.ams_home)
    }

    /**     OPTIONS MENU FUNCTIONS      **/
    override fun onCreateOptionsMenu(menu: Menu?): Boolean {
        val inflater: MenuInflater = menuInflater
        inflater.inflate(R.menu.main_menu, menu)
        return true
    }

    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        return when (item.itemId) {
            R.id.bluetooth_item -> {
                /**     START BLUETOOTH FRAGMENT TO PAIR    **/
                val toast = Toast.makeText(applicationContext, "Opening bluetooth page.", Toast.LENGTH_SHORT)
                toast.show()
                true
            }
            else -> super.onOptionsItemSelected(item)
        }
    }
}

