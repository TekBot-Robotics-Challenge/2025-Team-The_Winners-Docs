---
title: Team
template: splash

lastUpdated: 2025-09-30
head:
  - tag: style
    content: |
      .hero-bg { display: none !important; }
      .content-panel { max-width: 80rem !important; margin: auto; padding: 1.5rem 0px !important; }
      
      /* Title styling */
      h1 {
        text-align: center;
        font-size: 3.5rem;
        margin-bottom: 3rem;
        font-weight: 700;
        background: linear-gradient(135deg, var(--accent) 0%, #4f46e5 100%);
        -webkit-background-clip: text;
        -webkit-text-fill-color: transparent;
        background-clip: text;
      }
      
      h2 {
        font-size: 1.8rem;
        margin: 2.5rem 0 1.5rem;
        padding-bottom: 0.75rem;
        border-bottom: 3px solid;
        border-image: linear-gradient(90deg, var(--accent) 0%, #4f46e5 100%) 1;
        display: inline-block;
      }
      
      .section-header {
        text-align: center;
        width: 100%;
      }
      
      /* Team grid layout */
      .team-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
        gap: 1.5rem;
        margin-top: 1.5rem;
      }
      
      /* Team member card */
      .team-card {
        background: linear-gradient(145deg, rgba(var(--color-gray-700), 0.6) 0%, rgba(var(--color-gray-800), 0.8) 100%);
        border-radius: 16px;
        padding: 1.5rem 1rem;
        text-align: center;
        transition: all 0.3s ease;
        border: 1px solid rgba(var(--color-accent), 0.1);
        backdrop-filter: blur(10px);
      }
      
      .team-card:hover {
        transform: translateY(-8px);
        box-shadow: 0 20px 40px rgba(0, 0, 0, 0.3);
        border-color: var(--accent);
      }
      
      /* Team member image */
      .team-photo {
        width: 130px;
        height: 130px;
        border-radius: 50%;
        object-fit: cover;
        margin-bottom: 1rem;
        border: 4px solid var(--accent);
        padding: 3px;
        background: var(--color-bg);
        transition: all 0.3s ease;
        box-shadow: 0 4px 15px rgba(0, 0, 0, 0.2);
      }
      
      .team-card:hover .team-photo {
        transform: scale(1.05);
        box-shadow: 0 8px 25px rgba(var(--color-accent), 0.3);
      }
      
      /* Team member name */
      .team-name {
        font-size: 1rem;
        font-weight: 600;
        color: var(--color-text);
        margin-bottom: 0.25rem;
        line-height: 1.3;
      }
      
      /* Team member level */
      .team-level {
        font-size: 0.85rem;
        color: var(--accent);
        font-weight: 500;
      }
      
      /* Responsive adjustments */
      @media (max-width: 768px) {
        h1 { font-size: 2.5rem; }
        .team-grid { grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); }
        .team-photo { width: 110px; height: 110px; }
      }
---
## **Our Team**

<div class="section-header">

## Electronics Team

</div>

<div class="team-grid">

<div class="team-card">

![YEHOUENOU Peace Mathieu Chrysostome](/img/chr.jpg)

**YEHOUENOU Peace Mathieu Chrysostome**  
<span class="team-level">BAC + 4</span>

</div>

<div class="team-card">

![VIANOU Sèna Lucresse](/img/luc.jpg)

**VIANOU Sèna Lucresse**  
<span class="team-level">BAC + 4</span>

</div>

<div class="team-card">

![HOUNSA Kévin](/img/ke.jpg)

**HOUNSA Kévin**  
<span class="team-level">BAC + 4</span>

</div>

<div class="team-card">

![SONON Morel Sourou Pierre Claver](/img/mor.jpg)

**SONON Morel Sourou Pierre Claver**  
<span class="team-level">BAC + 4</span>

</div>

</div>

<div class="section-header">

## Computer Science Team

</div>

<div class="team-grid">

<div class="team-card">

![CHATIGRE Larissa Ashley Sèlomè](/img/lar.jpg)

**CHATIGRE Larissa Ashley Sèlomè**  
<span class="team-level">BAC + 4</span>

</div>

<div class="team-card">

![AGBODJA Marzoukath Foumi](/img/mazk.jpg)

**AGBODJA Marzoukath Foumi**  
<span class="team-level">BAC + 4</span>

</div>

<div class="team-card">

![COMLAN Ifè Léonce](/img/ife.jpg)

**COMLAN Ifè Léonce**  
<span class="team-level">BAC + 3</span>

</div>

</div>

<div class="section-header">

## Mechanical Team

</div>

<div class="team-grid">

<div class="team-card">

![GANTIN Philippe Junior Segnon](/img/ju.jpg)

**GANTIN Philippe Junior Segnon**  
<span class="team-level">BAC + 4</span>

</div>

<div class="team-card">

![NAMINAWADO Mikpamawu Géraud](/img/gr.jpg)

**NAMINAWADO Mikpamawu Géraud**  
<span class="team-level">BAC + 4</span>

</div>

<div class="team-card">

![DJIWAN Ariane](/img/ari.jpg)

**DJIWAN Ariane**  
<span class="team-level">BAC + 3</span>

</div>

</div>
